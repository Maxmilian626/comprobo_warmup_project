#!/usr/bin/env python
import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
from geometry_msgs.msg import Twist, Vector3
import numpy as np


class object_avoidance:
    def __init__(self):
        rospy.init_node('object_avoidance')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.rate = rospy.Rate(10)
        #self.bump_sub = rospy.Subscriber('/bump',Bump, self.bump_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        #self.speed = .3 #speed variable.
        self.turn = .5 #turn speed variable.
        self.distance = 50 #Target Reading from the lidar.
        self.diag_dist = np.cos(np.deg2rad(90)) * self.distance
        self.mode = 'simple' #Simple for dumb driving, advanced
                            #for advanced not bang bang driving.
                            #Todo: come up with said mythical behavior.

        self.state = '' #Follow right or left wall
            #angles, third one is positive or negative turn for
            #case where its too close
        self.ranges = []


    def act(self, theta, speed):
        #translates the direction received to a twist to publish.
        if (self.state == 'stop'):
            self.speed = 0
            self.turn = 0

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = theta*self.turn
        self.pub.publish(twist)

    def bump_callback(self, msg): #only happens when it gets a message.
        print "detected bump"
        if (msg.leftFront and msg.rightFront):
             self.state = 'stop'

    def scan_callback(self, msg):
        self.ranges = msg.ranges


    def mean_scan(self):
        measure_range = []
        #take the average of 5 degrees from
        for x in range(-5,5,1):
            index = 0
            if x < 0:
            #accounting for wraparound.
                index = 360 + x
            else:
                index = x

            if (len(self.ranges) > 359) and (self.ranges[index] != 0):
                #make sure there are 360 ranges, and that the front
                #isn't 0.
                measure_range.append(msg.ranges[index])

        mean_distance = np.mean(measure_range)
        return mean_distance

    def finder(self):
        if self.mode == 'simple':
            scan = self.mean_scan()

            if scan >= self.distance:
                #nothing in sight, drive forward.
                self.act(0, .3)
            if scan < self.distance:
                #something in sight, turn until no longer will run into thing.
                self.act(.3, 0)


        else if self.mode== 'advanced':
            pass


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.object_avoidance()
        r.sleep()

if __name__ == '__main__':
    node = wall_follow()
    node.run()
