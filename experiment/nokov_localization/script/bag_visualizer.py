#!/usr/bin/env python3
import rospy
import math
from visualization import Visualization
from sandbox_msgs.msg import Trajectory, TrajectoryPoint
from scipy.interpolate import CubicSpline
from vehicle_param import VehicleParams

visual: Visualization = None


def create_box(x, y, heading, length, width):
    dx1 = math.cos(heading) * length / 2
    dy1 = math.sin(heading) * length / 2
    dx2 = math.sin(heading) * width / 2
    dy2 = -math.cos(heading) * width / 2

    return [
        [x + dx1 + dx2, y + dy1 + dy2],
        [x + dx1 - dx2, y + dy1 - dy2],
        [x - dx1 - dx2, y - dy1 - dy2],
        [x - dx1 + dx2, y - dy1 + dy2],
        [x + dx1 + dx2, y + dy1 + dy2],
    ]


def publish_footprint(x, y, theta, phi):
    tire_radius = VehicleParams['tire_radius']
    front_tire_width = VehicleParams['tire_width']
    front_track_width = VehicleParams['width']
    rear_tire_width = VehicleParams['tire_width']
    rear_track_width = VehicleParams['width']

    front_pose_x = VehicleParams['wheel_base']
    rear_track_width_2 = rear_track_width / 2
    front_track_width_2 = front_track_width / 2
    box_length = tire_radius * 2
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)

    boxes = [
        create_box(x - rear_track_width_2 * sin_t - tire_radius, y + rear_track_width_2 * cos_t, theta,
                   box_length, rear_tire_width),
        create_box(x + rear_track_width_2 * sin_t - tire_radius, y - rear_track_width_2 * cos_t, theta,
                   box_length, rear_tire_width),
        create_box(x + front_pose_x - front_track_width_2 * sin_t, y + front_track_width_2 * cos_t, theta + phi,
                   box_length, front_tire_width),
        create_box(x + front_pose_x + front_track_width_2 * sin_t, y - front_track_width_2 * cos_t, theta + phi,
                   box_length, front_tire_width),
    ]



def traj_callback(msg: Trajectory):
    times = []
    xyt = []
    for point in msg.points:
        times.append(point.x)
        xyt.append([point.x, point.y, point.theta])

    traj_cs = CubicSpline(times, xyt)
    start_time = rospy.Time.now()

    def visual_traj(evt):
        elapsed_sec = (start_time - evt.current_real).to_sec()
        pose = traj_cs(elapsed_sec)


    timer = rospy.Timer(rospy.Duration.from_sec(0.01), visual_traj)
    timer.join()



if __name__ == '__main__':
    rospy.init_node('bag_visualizer')
    traj_sub = rospy.Subscriber('/traj', Trajectory, traj_callback, queue_size=1)
    visual = Visualization('/bag_markers', 'world')
    rospy.spin()
