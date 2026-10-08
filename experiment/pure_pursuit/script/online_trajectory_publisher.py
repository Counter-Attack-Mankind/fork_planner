#!/usr/bin/env python3
import json
import sys
import rospy
import rospkg
import bisect
import time
from std_msgs.msg import Empty, ColorRGBA, String
from nav_msgs.msg import Path
from sandbox_msgs.msg import AprilObject, Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation
import colorsys


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
    return s * (1 - t) + e * t


class TrajectoryListener:
    def __init__(self):
        self.traj_publisher = rospy.Publisher('/traj', Trajectory, queue_size=5, latch=True)
        self.visual_publisher = rospy.Publisher('/traj_visual', MarkerArray, queue_size=1, latch=True)
        self.truth_visual_publisher = rospy.Publisher('/truth_visual', MarkerArray, queue_size=1, latch=True)
        self.truth_publisher = rospy.Publisher('/truth', AprilObject, queue_size=1, latch=False)
        # self.traj_timer = rospy.Timer(rospy.Rate(1).sleep_dur, self.timer_callback)

        self.plan_sub = rospy.Subscriber('/plan', String, self.plan_callback, queue_size=1)
        self.start_time = -1
        self.time_profile = []
        self.vehicles = []

    def plan_callback(self, msg):
        fields = json.loads(msg.data)
        tf = fields['tf']
        solution = np.array(fields['solution'])
        nv, nfe, _ = solution.shape
        self.time_profile = np.linspace(0, tf, nfe)
        self.vehicles = []

        for v in range(nv):
            traj = {
                "x": [],
                "y": [],
                "yaw": [],
                "speed": [],
            }

            for i in range(nfe):
                traj['x'].append(solution[v, i, 0])
                traj['y'].append(solution[v, i, 1])
                traj['yaw'].append(solution[v, i, 2])
                traj['speed'].append(solution[v, i, 3])

            self.vehicles.append(traj)

        self.publish_traj()
        self.start_time = time.time()  # execute now
        self.visual_publisher.publish(visualize_trajectory(self.vehicles))
        self.truth_visual_publisher.publish(visualize_truth(self.vehicles, 0))

    def publish_traj(self):
        for (target, t) in enumerate(self.vehicles):
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

            self.traj_publisher.publish(traj)

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            t = time.time()
            if self.start_time > 0 and t - self.start_time < self.time_profile[-1]:
                relative_time = t - self.start_time
                index_right = bisect.bisect_right(self.time_profile, relative_time)
                index = max(0, index_right - 1)
                denom = (self.time_profile[index_right] - self.time_profile[index])
                lerp_t = (relative_time - self.time_profile[index]) / denom if denom > 0 else 0

                for (target, vehicle) in enumerate(self.vehicles):
                    if index >= len(vehicle['x']):
                        continue

                    obj = AprilObject()
                    obj.header.frame_id = 'world'
                    obj.header.stamp = rospy.Time.now()
                    obj.id = target

                    obj.x = lerp(vehicle['x'][index], vehicle['x'][index_right], lerp_t)
                    obj.y = lerp(vehicle['y'][index], vehicle['y'][index_right], lerp_t)
                    obj.yaw = lerp(vehicle['yaw'][index], vehicle['yaw'][index_right], lerp_t)
                    self.truth_publisher.publish(obj)

                self.truth_visual_publisher.publish(visualize_truth(self.vehicles, index))

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('trajectory_publisher')
    listener = TrajectoryListener()
    listener.spin()
