#!/usr/bin/env python3
import sys
import time
import rospy
from std_msgs.msg import Empty


if __name__ == '__main__':
    rospy.init_node('delayed_execute')

    execute_publisher = rospy.Publisher('/execute', Empty, queue_size=1, latch=False)
    delay = int(sys.argv[1]) if len(sys.argv) > 1 else 1

    def timer_callback(evt):
        execute_publisher.publish(Empty())
        rospy.signal_shutdown('fin')

    print('delay in %d secs' % delay)
    rospy.Timer(rospy.Duration(delay), timer_callback, True)
    rospy.spin()
