#!/usr/bin/env python
import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
from geometry_msgs.msg import Twist, Vector3
import numpy as np


class wall_follow:
    def __init__(self):
        rospy.init_node('wall_follow')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.rate = rospy.Rate(10)
        #self.bump_sub = rospy.Subscriber('/bump',Bump, self.bump_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.speed = .3 #speed variable.
        self.turn = .5 #turn speed variable.
        self.distance = 50 #Target Reading from the lidar.
        self.diag_dist = np.cos(np.deg2rad(90)) * self.distance
        self.mode = 'simple' #Simple for dumb driving, advanced
                            #for advanced not bang bang driving.
                            #Todo: come up with said mythical behavior.

        self.state = 'right_follow' #Follow right or left wall
        self.left_range = [45, 90, 135, 1]
        self.right_range = [315, 270, 225, -1]
            #angles, third one is positive or negative turn for
            #case where its too close
        self.ranges = []


    def act(self, theta):
        #translates the direction received to a twist to publish.
        if (self.state == 'stop'):
            self.speed = 0
            self.turn = 0

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = theta*self.turn
        self.pub.publish(twist)

    # def bump_callback(self, msg): #only happens when it gets a message.
    #     print "detected bump"
    #     if (msg.leftFront and msg.rightFront):
    #          self.state = 'stop'

    def scan_callback(self, msg):
        self.ranges = msg.ranges


    def get_range(self):
        if self.state = 'right_follow'
            return self.right_range
        else
            return self.left_range

    def wall_scan(self):
        #assuming that you begin relatively parallel to a wall.
        range_side = self.get_range() #get which side to follow
        #read the current readings
        top_read = self.range[range_side[0]]
        side_read = self.range[range_side[1]]
        bot_read = self.range[range_side[2]]
        direction = self.range[range_side[3]] #positive or negative
        #depending on which side it's going.

        if self.mode == 'simple':
            theta = 20 #amount of turn.
            if side_read < self.distance:
            #too close
                theta = theta*direction
            else if side_read == self.distance:
            #goldilocks
                theta = 0
            else if side_read > self.distance:
            #too far
                theta = -1*theta*direction
                self.act(theta)


        else if self.mode== 'advanced':
            pass


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.wall_scan()
        r.sleep()

if __name__ == '__main__':
    node = wall_follow()
    node.run()
