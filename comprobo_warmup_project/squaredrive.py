#!/usr/bin/env python
import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
from geometry_msgs.msg import Twist, Vector3
import time
import numpy as np


class square_drive:
    def __init__(self):
        rospy.init_node('square_drive')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.rate = rospy.Rate(10)
        self.bump_sub = rospy.Subscriber('/bump',Bump, self.bump_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.speed = .1 #speed variable.
        self.turn = .2 #turn variable.
        self.neato_size = 30. #cm
        self.drive_distance = 100. #cm
        self.turn_duration = 90. #duration to turn 90 degrees
        self.mode = 'advanced' #Simple for dumb driving, advanced
                            #for self-calibrating drive.

        self.state = 'new_run'
        self.key = ''
        self.scan_scale = 0 #1cm = x reading on the lidar.
        self.drive_duration_scale = 6 #1cm = drive for this amount of time
        self.turn_duration_scale = self.turn_duration/90. #per degree
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
        twist.linear.x = x*self.speed; twist.linear.y = y*self.speed; twist.linear.z = z*self.speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*self.turn
        self.pub.publish(twist)

    def bump_callback(self, msg): #only happens when it gets a message.

        if (msg.leftFront and msg.rightFront):
            self.state = 'cease'
            self.key = 'stop'
            self.act()

    def scan_callback(self, msg):
        self.ranges = msg.ranges
        #print self.ranges[0]


    def calibrate(self):
        #run forward until hitting a wall.
        while self.state != 'cease':
            self.key = 'forward'
            self.act()

        calibrate_range = []
        #hopefully we get receive a message from scan.
        distance = 0.
        for x in range(-5,5, 1):
            if (len(self.ranges) > 359) and (self.ranges[x] != 0):
                calibrate_range.append(self.ranges[x])
        if len(calibrate_range) != 0:
            distance = np.mean(calibrate_range)
        self.scan_scale = distance / self.neato_size

        self.speed = .2

        back_distance = 150. * self.scan_scale #Reading at 150 cm distance
        distance_duration = 0

        if self.ranges[0] != 0:
        #technically redundant until I implement a better "if 0 doesn't get me anything function"
            current_scan = self.ranges[0]
        else:
            current_scan = 0

        #while current_scan <= back_distance: #It seems to change the value of back_distance, distance, self.scan_scale, etc. just before it hits 0.
        while current_scan <= 1.6:
        #move backwards until the lidar reads the intended distance.
            self.state = "backwards"
            self.key = 'backwards'
            self.act()
            distance_duration += 1
            current_scan = self.ranges[0]


        self.state = 'calibrated' #previously calibrated was before going backwards, meaning that the
        #machine would read "calibrated" briefly, then go back to "cease" due to the bump sensors
        self.key = 'stop'
        self.act()

        self.drive_duration_scale = distance_duration / 150.

        turn_ref = distance
        temp_turn_duration = 0
        #turn until the 90 degree scan reads the same as the 0
        #degree scan previously did.

        #There's a huge honking blind spot at the 90ish range for whatever reason.
        #Need to be able to scan and calculate the diagonals for the equivalent distance.

        #actually, it's straight up skipping this again.
        current_scan = 0
        while not (.8*turn_ref  <= current_scan <= 1.2*turn_ref ):
        #Need some way to get it in the general range.
            self.key = 'right_turn'
            self.act()
            temp_turn_duration +=1

            current_scan = self.sweep(90)
            print turn_ref - current_scan

        self.turn_duration = temp_turn_duration

    def sweep(self, angle):
    #in case of blind spot, will sweep within 90 degrees of the target angle
    #to see if the diagonals agree on a certain distance from what the angle
    #is supposed to be, and then return the distance from the angle.
        bound = 90 #bound is the sweep angle from the target angle
        dist_collection = [] #array of calculated distances from diagonals.
        for x in (angle-bound/2, angle+bound/2, 1):
            index = x
            #normalize to 0-360
            if index < 0:
                index = 360 - x
            if index > 360:
                index = x - 360

            try:
                if (self.ranges[index] != 0):
                    dist_collection.append(self.ranges[index] * np.absolute(np.sin(np.deg2rad(index))))
            except: IndexError
                print "Didnt get that.  "

        avg_dist = np.mean(dist_collection)
        print avg_dist
        #Seems to send a whole bunch of the same number, and then it changes over to the next number.
        return avg_dist



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

        while not rospy.is_shutdown():
            if self.state != 'calibrated':
                self.calibrate()
                print "calibration complete" #runs it twice, apparently.  This changes the variables.  But why?  Because self.state is changed.
            if self.mode == 'advanced':
                self.square_up()
            elif self.mode == 'simple':
                self.simple_square()
            r.sleep()

if __name__ == '__main__':
    node = square_drive()
    node.run()
