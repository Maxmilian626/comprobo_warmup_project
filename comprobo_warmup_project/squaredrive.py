#!/usr/bin/env python
import rospy
from neato_node.msg import Bump
import sys, select, termios, tty
from geometry_msgs.msg import Twist, Vector3
import numpy as np


class square_drive:
    def __init__(self):
        rospy.init_node('square_drive')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.rate = rospy.Rate(10)
        self.bump_sub = rospy.Subscriber('/bump',Bump, self.bump_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.speed = .5 #speed variable.
        self.turn = 1.0 #turn variable.
        self.neato_size = 30 #cm
        self.drive_distance = 100 #cm
        self.turn_duration = 0


        self.state = 'new_run'
        self.key = ''
        self.scan_scale = 0 #1cm = x reading on the lidar.
        self.ranges = []




        #the neato will calibrate the lidar by running into a wall and
        #finding out the reading at the distance from the wall.  Since the
        #neato is a fixed length, we can then figure out how far to move
        #by scaling the distance with the measurements, and then using
        #the known distance and turning until the lidar picks up the same
        #reading roughly at a  90 degree angle.  We can avoid using time
        #based measurements with this.
        self.actions = { #different cases
        'forward':(1,0,0,0),
        'backwards':(-1,0,0,0),
        'left_turn':(0,0,0,1),
        'right_turn':(0,0,0,-1)
        'stop':(0,0,0,0)
        }


    def act(self):
        key =self.key
        if key in self.actions.keys():
            x = self.actions[key][0]
            y = self.actions[key][1]
            z = self.actions[key][2]
            th = self.actions[key][3]
        else:
            x = 0
            y = 0
            z = 0
            th = 0


        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        self.pub.publish(twist)

    def bump_callback(self, msg): #only happens when it gets a message.
        print "getting bump sensor"
        if any((msg.leftFront, msg.leftSide, msg.rightFront, msg.rightSide)):
             self.state = 'stop'

    def scan_callback(self, msg):
        self.ranges = msg.ranges


    def calibrate(self):
        #run forward until hitting a wall.
        while self.state != 'stop':
            self.key = 'forward'

        calibrate_range = []
        #hopefully we get receive a message from scan.
        for x in range(10):
            if (len(self.ranges) > 359) and (self.ranges[0] != 0)
            calibrate_range.append(msg.ranges)
        distance = np.mean(calibrate_range)
        self.scan_scale = distance / self.neato_size
        self.state = 'calibrated'

        back_distance = 150
        distance = 0
        while distance <= back_distance:
            self.key = 'backwards'
            self.act()
            distance += 1

        turn_ref = scan(0)
        temp_turn_duration = 0
        while temp_turn_duration <= turn_ref
        #Need some way to get it in the general range.
            self.key = 'right_turn'
            self.act()
            temp_turn_duration +=1
        self.turn_duration = temp_turn_duration


    def square_up(self):
        if self.state = 'calibrated':
            while (1): #should have an interrupt, but don't.  lol.
                self.drive_forward()
                self.drive_turn()

    def drive_forward(self):
        distance = 0
        while distance <= self.drive_distance
            self.key = 'forward'
            self.act()
            distance += 1

    def drive_turn(self):
        current_duration = 0
        while current_duration <= self.turn_duration
            self.key = 'right_turn'
            self.act()
            current_duration +=1


    def scan(self, angle):
        measure_range = []
        #hopefully we get receive a message from scan.
        for x in range(10):
            if (len(self.ranges) > 359) and (self.ranges[0] != angle)
            measure_range.append(msg.ranges)
        distance = np.mean(measure_range)
        return distance

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state != 'calibrated':
            self.calibrate()
            self.square_up()
        r.sleep()

if __name__ == '__main__':
    node = square_drive()
    node.run()
