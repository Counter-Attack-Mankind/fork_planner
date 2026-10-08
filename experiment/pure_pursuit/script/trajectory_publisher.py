#!/usr/bin/env python3

import sys
import rospy
import rospkg
import bisect
import time
from std_msgs.msg import Empty, ColorRGBA
from nav_msgs.msg import Path
from sandbox_msgs.msg import AprilObject, Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.io import loadmat
from scipy.spatial.transform import Rotation
from planner.reeds_shepp_path_planning import reeds_shepp_path_planning, plot_arrow
import colorsys


def generate_speed_profile(target_speed, directions):
    positions = []
    start = 0

    for i in range(1, len(directions)):
        if directions[i-1] != directions[i]:
            positions.append([start, i])
            start = i

    # fix last range
    positions.append([start, len(directions)])

    cs = CubicSpline([0.0, 0.2, 0.5, 0.8, 1.0], [0.0, target_speed * 0.8, target_speed, target_speed * 0.8, 0.0])
    profile = []

    for [start, end] in positions:
        profile += [cs(i) * directions[start] for i in np.linspace(0.0, 1.0, end - start)]

    return profile


def generate_time_profile(path_x, path_y, speed_profile):
    tp = [0.0]

    dx = np.diff(path_x, prepend=[path_x[0]])
    dy = np.diff(path_y, prepend=[path_y[0]])
    magnitude = np.sqrt(dx * dx + dy * dy)
    for (i, mag) in enumerate(magnitude):
        speed = abs(speed_profile[i])
        if speed < 1e-6:
            tp.append(tp[-1])
        else:
            tp.append(tp[-1] + mag / speed)

    return tp[1:]


def visualize_trajectory(vehicles):
    arr = MarkerArray()
    msg = Marker()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.ns = 'Markers'
    msg.id = 0

    msg.action = Marker.DELETEALL
    arr.markers.append(msg)

    vehicle_len = len(vehicles)
    for (target, vehicle) in enumerate(vehicles):
        msg = Marker()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.ns = 'Markers'
        msg.id = target + 1
        msg.action = Marker.ADD
        msg.type = Marker.LINE_STRIP
        msg.scale.x = 0.005
        msg.text = '# %d' % target

        for i in range(len(vehicle['x'])):
            pt = Point()
            pt.x = vehicle['x'][i]
            pt.y = vehicle['y'][i]
            msg.points.append(pt)

            color = ColorRGBA()
            (color.r, color.g, color.b) = colorsys.hsv_to_rgb(target / vehicle_len, abs(vehicle['speed'][i]) / 0.2, 1.0)
            color.a = 1.0
            msg.colors.append(color)

        arr.markers.append(msg)

    return arr


def visualize_truth(vehicle, index):
    arr = MarkerArray()
    msg = Marker()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.ns = 'Markers'
    msg.id = 0

    msg.action = Marker.DELETEALL
    arr.markers.append(msg)

    vehicle_len = len(vehicle)
    for (target, vehicle) in enumerate(vehicle):
        if index >= len(vehicle['x']):
            continue

        msg = Marker()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.ns = 'Markers'
        msg.id = target + 1
        msg.action = Marker.ADD
        msg.type = Marker.ARROW
        msg.scale.x = 0.05
        msg.scale.y = 0.01
        msg.scale.z = 0.03
        (msg.color.r, msg.color.g, msg.color.b) = colorsys.hsv_to_rgb(target / vehicle_len, 1.0, 1.0)
        msg.color.a = 1.0

        msg.pose.position.x = vehicle['x'][index]
        msg.pose.position.y = vehicle['y'][index]

        (msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w) = Rotation.from_euler('z', [vehicle['yaw'][index]]).as_quat()[0]

        arr.markers.append(msg)
    return arr

def lerp(s, e, t):
    return s * (1-t) + e * t

if __name__ == '__main__':
    vehicles = []
    time_profile = []

    rs_plan = 'plan' in sys.argv

    if rs_plan:
        start_x = 0.2  # [m]
        start_y = 0.6  # [m]
        start_yaw = np.deg2rad(0.0)  # [rad]

        end_x = 1.2  # [m]
        end_y = 0.0  # [m]
        end_yaw = np.deg2rad(90.0)  # [rad]

        curvature = 1.0
        step_size = 0.005

        path_x, path_y, path_yaw, mode, path_length, directions = reeds_shepp_path_planning(
            start_x, start_y, start_yaw,
            end_x, end_y, end_yaw, curvature, step_size)

        speed_profile = generate_speed_profile(0.30, directions)
        time_profile = generate_time_profile(path_x, path_y, speed_profile)

        vehicles.append({
            "x": path_x,
            "y": path_y,
            "yaw": path_yaw,
            "speed": speed_profile,
        })
    else:
        mat = loadmat(rospkg.RosPack().get_path('pure_pursuit') + '/script/Openloopresult.mat')
        data = mat['ouyang']
        (nrows, ncols) = data.shape
        vehicle_count = int((ncols - 1) / 4)

        for i in range(1, nrows):
            time_profile.append(data[i, 0])

        for v in range(vehicle_count):
            traj = {
                "x": [],
                "y": [],
                "yaw": [],
                "speed": [],
                "time": [],
            }

            for i in range(1, nrows):
                traj['x'].append(data[i, v * 4 + 1])
                traj['y'].append(data[i, v * 4 + 2])
                traj['yaw'].append(data[i, v * 4 + 3])
                traj['speed'].append(data[i, v * 4 + 4])

            # sample_size = 3
            # traj['x'] = np.array(traj['x'])[np.linspace(0, len(traj['x'])-1, sample_size, dtype='int')]
            # traj['y'] = np.array(traj['y'])[np.linspace(0, len(traj['y'])-1, sample_size, dtype='int')]
            # traj['yaw'] = np.array(traj['yaw'])[np.linspace(0, len(traj['yaw'])-1, sample_size, dtype='int')]
            # traj['speed'] = np.array(t    raj['speed'])[np.linspace(0, len(traj['speed'])-1, sample_size, dtype='int')]
            # time_profile = np.array(time_profile)[np.linspace(0, len(time_profile)-1, sample_size, dtype='int')]
            vehicles.append(traj)

    show_plot = 'plot' in sys.argv

    if show_plot:
        for (target, traj) in enumerate(vehicles):
            plt.plot(traj['x'], traj['y'], label=("Trajectory %d" % target))

            # plotting
            plot_arrow(traj['x'][0], traj['y'][0], traj['yaw'][0], fc='w')
            plot_arrow(traj['x'][-1], traj['y'][-1], traj['yaw'][-1])

        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        plt.show()
        exit(0)

    rospy.init_node('trajectory_publisher')

    traj_publisher = rospy.Publisher('/traj', Trajectory, queue_size=5, latch=True)
    visual_publisher = rospy.Publisher('/traj_visual', MarkerArray, queue_size=1, latch=True)
    truth_visual_publisher = rospy.Publisher('/truth_visual', MarkerArray, queue_size=1, latch=True)
    truth_publisher = rospy.Publisher('/truth', AprilObject, queue_size=1, latch=False)

    start_time = -1

    def execute_callback(msg):
        global start_time
        start_time = time.time()

    execute_subscriber = rospy.Subscriber('/execute', Empty, execute_callback, queue_size=1)


    def timer_callback(evt):
        for (target, t) in enumerate(vehicles):
            traj = Trajectory()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = 'world'
            traj.target = target

            path = Path()
            path.header = traj.header

            for i in range(len(t['x'])):
                pt = TrajectoryPoint()
                pt.x = t['x'][i]
                pt.y = t['y'][i]
                pt.yaw = t['yaw'][i]
                pt.velocity = t['speed'][i]
                pt.acceleration = 0.0
                traj.points.append(pt)

                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = pt.x
                pose.pose.position.y = pt.y
                path.poses.append(pose)

            traj_publisher.publish(traj)
    traj_timer = rospy.Timer(rospy.Rate(1).sleep_dur, timer_callback)
    visual_publisher.publish(visualize_trajectory(vehicles))
    truth_visual_publisher.publish(visualize_truth(vehicles, 0))

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        t = time.time()
        if start_time > 0 and t - start_time < time_profile[-1]:
            relative_time = t - start_time
            index_right = bisect.bisect_right(time_profile, relative_time)
            index = max(0, index_right - 1)
            denom = (time_profile[index_right] - time_profile[index])
            lerp_t = (relative_time - time_profile[index]) / denom if denom > 0 else 0

            for (target, vehicle) in enumerate(vehicles):
                if index >= len(vehicle['x']):
                    continue

                obj = AprilObject()
                obj.header.frame_id = 'world'
                obj.header.stamp = rospy.Time.now()
                obj.id = target

                obj.x = lerp(vehicle['x'][index], vehicle['x'][index_right], lerp_t)
                obj.y = lerp(vehicle['y'][index], vehicle['y'][index_right], lerp_t)
                obj.yaw = lerp(vehicle['yaw'][index], vehicle['yaw'][index_right], lerp_t)
                truth_publisher.publish(obj)

            truth_visual_publisher.publish(visualize_truth(vehicles, index))

        rate.sleep()
