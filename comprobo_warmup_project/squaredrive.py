#!/usr/bin/env python
import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
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

        self.speed = .3 #speed variable.
        self.turn = .5 #turn variable.
        self.neato_size = 30 #cm
        self.drive_distance = 100 #cm
        self.turn_duration = 90 #duration to turn 90 degrees
        self.mode = 'simple' #Simple for dumb driving, advanced
                            #for self-calibrating drive.

        self.state = 'new_run'
        self.key = ''
        self.scan_scale = 0 #1cm = x reading on the lidar.
        self.drive_duration_scale = 6 #1cm = drive for this amount of time
        self.turn_duration_scale = self.turn_duration/90 #per degree
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
        'right_turn':(0,0,0,-1),
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
        if (msg.leftFront and msg.rightFront):
             self.state = 'cease'
             self.key = 'stop'

    def scan_callback(self, msg):
        self.ranges = msg.ranges


    def calibrate(self):
        #run forward until hitting a wall.
        while self.state != 'cease':
            self.key = 'forward'

        calibrate_range = []
        #hopefully we get receive a message from scan.
        for x in range(-5,5, 1):
            if (len(self.ranges) > 359) and (self.ranges[x] != 0):
                calibrate_range.append(msg.ranges[x])
        distance = np.mean(calibrate_range)
        self.scan_scale = distance / self.neato_size
        self.state = 'calibrated'

        back_distance = 150 * self.scan_scale #Reading at 150 cm distance
        distance_duration = 0
        if self.ranges[0] != 0:
            current_scan = self.ranges[0]
        else:
            current_scan = 0

        while current_scan <= back_distance:
            self.key = 'backwards'
            self.act()
            distance_duration += 1
            current_scan = self.ranges[0]
            print self.key

        self.drive_duration_scale = distance_duration / 150

        turn_ref = distance
        temp_turn_duration = 0
        #turn until the 90 degree scan reads the same as the 0
        #degree scan previously did.
        if self.ranges[90] != 0:
            current_scan = self.ranges[90]
        else:
            current_scan = 0

        while not (turn_ref - 10 <= current_scan <= turn_ref + 10):
        #Need some way to get it in the general range.
            self.key = 'right_turn'
            self.act()
            temp_turn_duration +=1
            self.current_scan = self.ranges[90]
        self.turn_duration = temp_turn_duration

    def square_up(self):
        if self.state == 'calibrated':
            self.drive_forward(100) #drive one meter.
            self.drive_turn(self.turn_duration)


    def drive_forward(self, end_distance):
        distance = 0
        while (distance <= self.drive_duration_scale * end_distance):
            self.key = 'forward'
            self.act()
            distance += 1

    def drive_turn(self, angle):
        current_duration = 0
        while (current_duration <= angle * self.turn_duration_scale):
            self.key = 'left_turn'
            self.act()
            current_duration +=1


    def simple_square(self):
        self.drive_forward(100)
        self.drive_turn(90)


    def run(self):
        r = rospy.Rate(10)
        if self.state != 'calibrated':
            self.calibrate()
        while not rospy.is_shutdown():
            if self.mode == 'advanced':
                self.square_up()
            else if self.mode == 'simple':
                self.simple_square()
        r.sleep()

if __name__ == '__main__':
    node = square_drive()
    node.run()
