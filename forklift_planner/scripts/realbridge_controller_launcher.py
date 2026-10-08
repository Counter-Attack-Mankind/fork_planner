#!/usr/bin/env python3

"""Start one existing controller launch for each selected physical vehicle."""

import os

import roslaunch
import rospkg
import rospy


def parse_vehicle_ids(value):
    text = str(value)
    ids = []
    for part in text.split(','):
        token = part.strip()
        if not token:
            raise ValueError("vehicle_ids contains an empty item")
        try:
            vehicle_id = int(token)
        except ValueError as exc:
            raise ValueError("vehicle_ids must be comma-separated integers") from exc
        if vehicle_id < 0 or vehicle_id > 7:
            raise ValueError("vehicle_ids must be in [0,7]")
        if vehicle_id in ids:
            raise ValueError("vehicle_ids must not contain duplicates")
        ids.append(vehicle_id)
    if not ids:
        raise ValueError("vehicle_ids must not be empty")
    return ids


def main():
    rospy.init_node('realbridge_controller_launcher')
    try:
        vehicle_ids = parse_vehicle_ids(
            rospy.get_param('/forklift_planner/multi_vehicle/vehicle_ids'))
    except (KeyError, ValueError) as exc:
        rospy.logfatal('Invalid real-mode vehicle_ids: %s', exc)
        raise

    controller = rospy.get_param('~controller', 'pp')
    if controller not in ('pp', 'lqr'):
        raise ValueError("controller must be 'pp' or 'lqr'")
    lqr_q = rospy.get_param('~lqr_q', 1.0)
    debug_log_dir = rospy.get_param('~debug_log_dir', '')

    launch_file = os.path.join(
        rospkg.RosPack().get_path('forklift_planner'),
        'launch', 'one_controller.launch')
    launch_files = []
    for vehicle_id in vehicle_ids:
        args = [
            'target:={}'.format(vehicle_id),
            'controller:={}'.format(controller),
            'lqr_q:={}'.format(lqr_q),
        ]
        if debug_log_dir:
            args.append('debug_log_dir:={}'.format(debug_log_dir))
        launch_files.append((launch_file, args))

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    rospy.loginfo('Starting %s controllers for vehicle IDs: %s',
                  controller, ','.join(str(item) for item in vehicle_ids))
    parent.start()
    try:
        rospy.spin()
    finally:
        parent.shutdown()


if __name__ == '__main__':
    main()
