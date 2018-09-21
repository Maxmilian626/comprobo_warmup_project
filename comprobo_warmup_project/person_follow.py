#!/usr/bin/env python
import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import operator

class person_follow:
    def __init__(self):
        rospy.init_node('person_follow')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.rate = rospy.Rate(10)
        #self.bump_sub = rospy.Subscriber('/bump',Bump, self.bump_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.speed = .3 #speed variable.
        self.turn = .5 #turn speed variable.
        self.distance = 20 #Distance to stop.
        self.diag_dist = np.cos(np.deg2rad(90)) * self.distance
        self.mode = 'simple' #Simple for dumb driving, advanced
                            #for advanced not bang bang driving.
                            #Todo: come up with said mythical behavior.

        self.clust_thr = 5 #Cluster threshold to be considered a cluster

        self.state = '' #tells robot what the general command is.
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

    # def mean_scan(self):
    #     measure_range = []
    #     #take the average of 5 degrees from
    #     for x in range(-5,5,1):
    #         index = 0
    #         if x < 0:
    #         #accounting for wraparound.
    #             index = 360 + x
    #         else:
    #             index = x
    #
    #         if (len(self.ranges) > 359) and (self.ranges[index] != 0):
    #             #make sure there are 360 ranges, and that the front
    #             #isn't 0.
    #             measure_range.append(msg.ranges[index])
    #
    #     mean_distance = np.mean(measure_range)
    #     return mean_distance

    def find_clusters(self):
        #Cluster by determining if adjacent scan has a similar distance.
        #populate a list with clusters.
        #Clusters are a tuple of index (angle in degrees), distance, and size.
        cluster_list = []
        temp_cluster_points = [] #houses all the points of a cluster.
        temp_cluster_index = []
        init_dist = 0 #initial comparison.
        for i in self.ranges:
            if (init_dist - clust_thr < i < init_dist + clust_thr):
            #check if within range,
                temp_cluster_point.append(i)
                temp_cluster_index.append(self.ranges.index(i))
                #add point to cluster.
            else:
                init_dist = self.ranges
                if (len(temp_cluster_points) > 1):
                #tuple creation
                    index = np.median(temp_cluster_index)
                    distance = self.ranges[index]
                    size = len(temp_cluster_points)
                    data = (index, distance, size)
                    cluster_list.append(data)

                temp_cluster_points = []
                temp_cluster_index = []

        return cluster_list
    def navigate(self):
        if self.mode == 'simple':
            #Find out the direction to the closest person (center of cluster),
            #and then drive towards it.
            data = find_clusters()
            direction = min(data, key=operator.itemgetter(1))
            #Get shortest distance thing.   Preferably a person.
            #direction = (index, distance, size)
            if direction[1] <= self.distance
                self.state = 'stop'
            else:
                if ()-5 < direction[0] < 5):
                #within 5 degrees
                    self.act(0, .3)
                else:
                    turn_dir = np.sign(np.sin(np.deg2rad(direction[0])))
                    #positive if 0-180, negative otherwise
                    self.act(turn_dir, 0)


        else if self.mode== 'advanced':
            pass


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.navigate()
        r.sleep()

if __name__ == '__main__':
    node = person_follow()
    node.run()
