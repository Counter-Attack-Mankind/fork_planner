import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from enum import Enum
from scipy.spatial.transform import Rotation


class ScatterType(Enum):
    Cube = Marker.CUBE_LIST
    Sphere = Marker.SPHERE_LIST
    Point = Marker.POINTS


def convert_color(color):
    if len(color) == 4:
        return ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
    elif len(color) == 3:
        return ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)


class Visualization:
    def __init__(self, topic, frame_id):
        self.pub = rospy.Publisher(topic, MarkerArray, queue_size=100, latch=True)
        self.frame_id = frame_id

    def plot(self, x, y, width=0.1, color=(1.0, 1.0, 1.0), ns="plot", id=1, close=False):
        if len(x) != len(y) or len(x) == 0:
            return

        msg = MarkerArray()
        marker = Marker(ns=ns, id=id, action=Marker.ADD, type=Marker.LINE_STRIP)
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation.w = 1.0
        marker.scale.x = width

        marker.points = [Point(x=x[i], y=y[i], z=0.1) for i, _ in enumerate(x)]
        if close:
            marker.points.append(marker.points[0])

        if isinstance(color, list):
            marker.colors = [convert_color(c) for c in color]
        else:
            marker.color = convert_color(color)

        msg.markers.append(marker)
        self.pub.publish(msg)

    def scatter(self, x, y, size=0.2, color=(1.0, 1.0, 1.0), type=ScatterType.Point, ns="scatter", id=1):
        if len(x) != len(y) or len(x) == 0:
            return

        msg = MarkerArray()
        marker = Marker(ns=ns, id=id, action=Marker.ADD, type=type.value)
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = size

        marker.points = [Point(x=x[i], y=y[i], z=0.2) for i, _ in enumerate(x)]

        if isinstance(color, list):
            marker.colors = [convert_color(c) for c in color]
        else:
            marker.color = convert_color(color)

        msg.markers.append(marker)
        self.pub.publish(msg)

    def arrow(self, x, y, theta, width=0.005, length=0.03, color=(1.0, 1.0, 1.0), ns="arrow", id=1):
        msg = MarkerArray()
        marker = Marker(ns=ns, id=id, action=Marker.ADD, type=Marker.ARROW)
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w \
            = Rotation.from_euler('xyz', [0, 0, theta]).as_quat()
        marker.scale.x = length
        marker.scale.y = width * 1.5
        marker.scale.z = width

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        marker.color = convert_color(color)

        msg.markers.append(marker)
        self.pub.publish(msg)
