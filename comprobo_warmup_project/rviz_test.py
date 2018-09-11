#!/usr/bin/env python

#http://docs.ros.org/indigo/api/rviz_python_tutorial/html/
import rospy
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
import rviz
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

class marker():
    def __init__(self):
        rospy.init_node('sphere_marker')
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            marker = Marker()

            marker.type = Marker.SPHERE
            marker.pose.position.x = 1
            marker.pose.position.y = 2

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.pub.publish(marker)
            r.sleep()

if __name__ == '__main__':
    node = marker()
    node.run()
