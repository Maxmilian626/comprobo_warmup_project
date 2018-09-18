#!/usr/bin/env python
import rospy
from neato_node.msg import Bump
import sys, select, termios, tty
from geometry_msgs.msg import Twist, Vector3




class wall_follow:
    def __init__(self):
        rospy.init_node('estop')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.rate = rospy.Rate(10)
        self.sub = rospy.Subscriber('/bump',Bump, self.callback)
        self.speed = .5 #speed variable.
        self.turn = 1.0 #turn variable.

        self.switch = { #different cases
        'forward':(1,0,0,0),
        'backwards':(-1,0,0,0),
        'left_turn':(0,0,0,1),
        'stop':(0,0,0,0)
        }

        self.state = 'forward'

    def process_callback(self):
            #switch it to simply one of the cases.
        speed = self.speed
        turn = self.turn
        key = self.state #callback will give the state.
        print key

        #if (self.getKey() =='\x03'): #ctrl^C to quit.
        #    key = 'stop'
        if key in self.switch.keys():
            x = self.switch[key][0]
            y = self.switch[key][1]
            z = self.switch[key][2]
            th = self.switch[key][3]
        else:
            x = 0
            y = 0
            z = 0
            th = 0


        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        self.pub.publish(twist)

    def callback(self, msg): #only happens when it gets a message.
        print "getting bump sensor"
        if any((msg.leftFront, msg.leftSide, msg.rightFront, msg.rightSide)):
             self.state = 'stop'
             print "stop"

    def getKey(self):
    	tty.setraw(sys.stdin.fileno())
    	select.select([sys.stdin], [], [], 0)
    	key = sys.stdin.read(1)
    	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.process_callback()
        r.sleep()

if __name__ == '__main__':
    node = estop()
    node.run()
