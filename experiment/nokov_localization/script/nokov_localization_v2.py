#!/usr/bin/env python3
"""
简化版 Nokov 定位节点（适配新版 Nokov 软件）。

新版 Nokov 软件可以直接将后轮中心反光点设为刚体定位点，
因此不再需要多 marker 推算后轮中心位置。
每个车辆只需订阅刚体 Pose 即可获取 (x, y, yaw)。

用法：
    rosrun nokov_localization nokov_localization_v2.py
"""
import json
import math
import time

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from jsk_recognition_msgs.msg import PolygonArray
from functools import partial

from visualization import Visualization, ScatterType
from vehicle_param import VehicleParams
from sandbox_msgs.msg import AprilObject

# 与 C++ Color::colors[i+1] 一致（vehicle 0~7）
VEHICLE_COLORS = [
    (0.0, 0.0, 1.0),    # 0: Blue
    (0.3, 0.3, 0.3),    # 1: Grey
    (1.0, 0.0, 0.0),    # 2: Red
    (0.0, 1.0, 0.0),    # 3: Green
    (1.0, 0.55, 0.0),   # 4: Orange
    (0.78, 0.5, 0.3),   # 5: Brown
    (0.5, 0.0, 0.0),    # 6: Maroon
    (1.0, 0.0, 1.0),    # 7: Magenta
]


def compute_footprint(x, y, yaw):
    """根据后轮中心位置和朝向计算车辆足迹四角"""
    wb = VehicleParams['wheel_base']
    fh = VehicleParams['front_hang']
    rh = VehicleParams['rear_hang']
    hw = VehicleParams['width'] / 2
    mh = VehicleParams['marker_height']

    local = [
        (-rh, -hw), (wb + fh, -hw),
        (wb + fh,  hw), (-rh,  hw),
    ]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    corners = []
    for lx, ly in local:
        corners.append((
            x + lx * cos_y - ly * sin_y,
            y + lx * sin_y + ly * cos_y,
        ))
    return corners


class Localization:
    def __init__(self, ns):
        self.vehicles = {}      # vid -> PoseStamped (最新刚体位姿)
        self.boundary_pts = {}  # marker_idx -> PoseStamped
        self.obstacles = {}     # obs_idx -> {marker_idx: PoseStamped}

        self.visual = Visualization('nokov_markers', 'world')
        self.info_pub = rospy.Publisher('/nokov_info', String, queue_size=10, latch=True)
        self.object_pub = rospy.Publisher('/object', AprilObject, queue_size=10, latch=True)
        self.polygon_pub = rospy.Publisher('/obstacles', PolygonArray, queue_size=10, latch=True)

        # 自动发现 VRPN 话题并订阅
        topics = rospy.get_published_topics(ns)
        for topic, _ in topics:
            parts = topic[1:].split('/')
            if len(parts) != 3:
                continue
            _, name, sub_topic = parts
            if sub_topic != 'pose':
                continue

            if name.startswith('Boundary'):
                # 边界标记：Boundary_Marker1, Boundary_Marker2, ...
                try:
                    idx = int(name.split('Marker')[-1])
                except (ValueError, IndexError):
                    idx = 0
                self.boundary_pts[idx] = None
                rospy.Subscriber(topic, PoseStamped,
                                 partial(self._boundary_cb, idx), queue_size=1)

            elif name.startswith('Vehicle'):
                # 车辆刚体：Vehicle0, Vehicle1, ...
                # 跳过单个 marker 话题（Vehicle0_Marker1）
                if '_' in name:
                    continue
                try:
                    vid = int(name.replace('Vehicle', ''))
                except ValueError:
                    continue
                self.vehicles[vid] = None
                rospy.Subscriber(topic, PoseStamped,
                                 partial(self._vehicle_cb, vid), queue_size=1)
                rospy.loginfo('订阅车辆 %d: %s', vid, topic)

            elif name.startswith('Obstacle'):
                if '_' not in name:
                    continue
                parts_name = name.split('_')
                try:
                    obs_idx = int(parts_name[0].replace('Obstacle', ''))
                    marker_idx = int(parts_name[1].replace('Marker', ''))
                except (ValueError, IndexError):
                    continue
                self.obstacles.setdefault(obs_idx, {})[marker_idx] = None
                rospy.Subscriber(topic, PoseStamped,
                                 partial(self._obstacle_cb, obs_idx, marker_idx),
                                 queue_size=1)

        rospy.loginfo('发现 %d 辆车, %d 个边界点, %d 个障碍物',
                      len(self.vehicles), len(self.boundary_pts), len(self.obstacles))

    # ── 回调 ──────────────────────────────────────────────
    def _vehicle_cb(self, vid, msg):
        self.vehicles[vid] = msg

    def _boundary_cb(self, idx, msg):
        self.boundary_pts[idx] = msg

    def _obstacle_cb(self, obs_idx, marker_idx, msg):
        self.obstacles[obs_idx][marker_idx] = msg

    # ── 工具方法 ──────────────────────────────────────────
    @staticmethod
    def _pose_to_xyyaw(msg):
        """从 PoseStamped 提取 (x, y, yaw)"""
        pos = msg.pose.position
        ori = msg.pose.orientation
        _, _, yaw = Rotation.from_quat(
            [ori.x, ori.y, ori.z, ori.w]).as_euler('xyz')
        return pos.x, pos.y, yaw

    # ── 主循环 ────────────────────────────────────────────
    def spin_once(self):
        loc_info = {
            'boundary': [],
            'obstacles': [],
            'vehicles': {},
        }

        # ─── 边界 ───
        bnd_points = np.array([
            [v.pose.position.x, v.pose.position.y]
            for v in self.boundary_pts.values() if v is not None
        ])
        if bnd_points.shape[0] > 0:
            bnd_m = bnd_points / 1000.0  # 可视化用米
            self.visual.scatter(bnd_m[:, 0], bnd_m[:, 1],
                                size=0.02, type=ScatterType.Sphere, ns='boundary points')
        if bnd_points.shape[0] >= 2:
            bmin = np.min(bnd_points, axis=0)
            bmax = np.max(bnd_points, axis=0)
            loc_info['boundary'] = [float(bmin[0]), float(bmax[0]),
                                    float(bmin[1]), float(bmax[1])]
            # 可视化用米
            rect_m = np.array([
                [bmin[0], bmin[1]], [bmin[0], bmax[1]],
                [bmax[0], bmax[1]], [bmax[0], bmin[1]],
            ]) / 1000.0
            self.visual.plot(rect_m[:, 0], rect_m[:, 1], width=0.01,
                             color=(1.0, 1.0, 1.0, 0.2), ns='boundary', close=True)

        # ─── 障碍物 ───
        polygons = PolygonArray()
        polygons.header.stamp = rospy.Time.now()
        polygons.header.frame_id = 'world'
        for obs_idx, markers in self.obstacles.items():
            pts = np.array([
                [v.pose.position.x, v.pose.position.y]
                for v in markers.values() if v is not None
            ])
            if pts.shape[0] > 2:
                hull = ConvexHull(pts)
                vertices = pts[hull.vertices]
                loc_info['obstacles'].append(vertices.tolist())
                self.visual.plot(vertices[:, 0], vertices[:, 1], width=0.01,
                                 color=(0.9, 0, 0.7), ns='obstacle', id=obs_idx, close=True)
                poly = PolygonStamped()
                poly.header = polygons.header
                poly.polygon.points = [Point32(x=v[0], y=v[1], z=0.0) for v in vertices]
                polygons.polygons.append(poly)
        if polygons.polygons:
            self.polygon_pub.publish(polygons)

        # ─── 车辆 ───
        for vid, pose_msg in sorted(self.vehicles.items()):
            if pose_msg is None:
                continue

            x, y, yaw = self._pose_to_xyyaw(pose_msg)

            # 发布 AprilObject
            april_msg = AprilObject(
                type=AprilObject.VEHICLE, id=vid, x=x, y=y, yaw=yaw)
            april_msg.header.stamp = rospy.Time.now()
            april_msg.header.frame_id = 'world'
            self.object_pub.publish(april_msg)

            loc_info['vehicles'][vid] = (x, y, yaw)

            # RViz 可视化：用米坐标（/1000），与规划器可视化对齐
            xm, ym = x / 1000.0, y / 1000.0
            color = VEHICLE_COLORS[vid % len(VEHICLE_COLORS)]
            corners = compute_footprint(xm, ym, yaw)
            cx = [c[0] for c in corners]
            cy = [c[1] for c in corners]
            self.visual.plot(cx, cy, width=0.01, ns='vehicle',
                             color=color, id=vid, close=True)
            self.visual.arrow(xm, ym, yaw, color=color, length=0.05, id=vid)

        self.info_pub.publish(String(data=json.dumps(loc_info)))


if __name__ == '__main__':
    rospy.init_node('nokov_localization')
    ns = '/vrpn_client_node'
    time.sleep(2)
    loc = Localization(ns)
    timer = rospy.Timer(rospy.Duration.from_sec(1.0 / 60.0), lambda msg: loc.spin_once())
    rospy.spin()
