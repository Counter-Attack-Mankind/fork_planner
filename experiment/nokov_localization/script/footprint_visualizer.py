#!/usr/bin/env python3
import rospy
import math
import numpy as np
from visualization import Visualization
from vehicle_param import VehicleParams
from sandbox_msgs.msg import ChassisCommand, AprilObject

visual: Visualization = None
gpses = []
current_phi = 0.0


def create_box(x, y, heading, length, width):
    dx1 = math.cos(heading) * length / 2
    dy1 = math.sin(heading) * length / 2
    dx2 = math.sin(heading) * width / 2
    dy2 = -math.cos(heading) * width / 2

    return np.array([
        [x + dx1 + dx2, y + dy1 + dy2],
        [x + dx1 - dx2, y + dy1 - dy2],
        [x - dx1 - dx2, y - dy1 - dy2],
        [x - dx1 + dx2, y - dy1 + dy2],
        [x + dx1 + dx2, y + dy1 + dy2],
    ])


def chassis_callback(msg: ChassisCommand):
    global current_phi
    current_phi = min(25 / 180 * math.pi, max(-25 / 180 * math.pi, msg.steering))


def publish_tires(x, y, theta, phi):
    tire_radius = VehicleParams['tire_radius']
    front_tire_width = VehicleParams['tire_width']
    front_track_width = VehicleParams['width'] - front_tire_width
    rear_tire_width = VehicleParams['tire_width']
    rear_track_width = VehicleParams['width'] - rear_tire_width

    wl = VehicleParams['wheel_base']
    rear_track_width_2 = rear_track_width / 2
    front_track_width_2 = front_track_width / 2
    box_length = tire_radius * 2
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)

    boxes = [
        create_box(x - rear_track_width_2 * sin_t, y + rear_track_width_2 * cos_t, theta,
                   box_length, rear_tire_width),
        create_box(x + rear_track_width_2 * sin_t, y - rear_track_width_2 * cos_t, theta,
                   box_length, rear_tire_width),
        create_box(x + wl * cos_t - front_track_width_2 * sin_t, y + wl * sin_t + front_track_width_2 * cos_t, theta + phi,
                   box_length, front_tire_width),
        create_box(x + wl * cos_t + front_track_width_2 * sin_t, y + wl * sin_t - front_track_width_2 * cos_t, theta + phi,
                   box_length, front_tire_width),
    ]

    for (i, box) in enumerate(boxes):
        visual.plot(box[:, 0], box[:, 1], width=0.004, color=(0.5, 0.2, 1), id=i, ns='tires')


def object_callback(msg: AprilObject):
    gpses.append([msg.x, msg.y])
    npgps = np.array(gpses)
    visual.plot(npgps[:, 0], npgps[:, 1], width=0.01, color=(1.0, 0.0, 0.0), ns='localization')
    publish_tires(msg.x, msg.y, msg.yaw, current_phi)


if __name__ == '__main__':
    rospy.init_node('bag_visualizer')
    traj_sub = rospy.Subscriber('/chassis', ChassisCommand, chassis_callback, queue_size=1)
    object_sub = rospy.Subscriber('/object', AprilObject, object_callback, queue_size=1)
    visual = Visualization('/bag_markers', 'world')
    rospy.spin()
