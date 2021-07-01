#!/usr/bin/env python
import rospy
# import sys
# from std_msgs.msg import String
# from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math as Math

from math import pi
arena_map = np.zeros((20, 20))


print(arena_map)


class mapperClass:

    def __init__(self):
        self.front = 0
        self.left = 0
        self.right = 0
        self.direction = 'north'
        self.prevState = 0
        self.state = 0
        self.linear = 1
        self.angular = 0
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.poseX = 0
        self.poseY = 0

        self.orientation = 0

        rospy.Subscriber("/sensor/ir_left", Range, self.subscriber)
        rospy.Subscriber("/sensor/ir_right", Range, self.subscriber)
        rospy.Subscriber("/sensor/ir_front", Range, self.subscriber)

        rospy.Subscriber('/odom', Odometry, self.setOdom)

    def setOdom(self, msg):
        self.poseX = msg.pose.pose.position.x

        self.poseY = msg.pose.pose.position.y

        print("POSE X : ", self.poseX)
        print("POSE Y : ", self.poseY)
        arena_map[Math.floor(self.poseX + 10)][Math.floor(self.poseY + 10)] = 1

        if(self.left == 2):
            if(self.poseY + 10 + self.left >= 20):
                arena_map[Math.floor(self.poseX + 10)
                          ][Math.floor(self.poseY + 10 - self.left)] = 3
            else:
                arena_map[Math.floor(self.poseX + 10)
                          ][Math.floor(self.poseY + 10 - self.left)] = 3

        if(self.right == 2):
            if(self.poseY + 10 + self.right >= 20):
                arena_map[Math.floor(self.poseX + 10)
                          ][Math.floor(self.poseY + 10 - self.right)] = 2
            else:
                arena_map[Math.floor(self.poseX + 10)
                          ][Math.floor(self.poseY + 10 - self.right)] = 1
        print(
            f'Left Calc : {Math.floor(self.poseY + 10 + self.left)} | {Math.floor(self.poseY + 10 - self.left)} ')
        print(
            f'Right Calc : {Math.floor(self.poseY + 10 + self.right)} | {Math.floor(self.poseY + 10 - self.right)} ')

        print(arena_map)

    def subscriber(self, msg):
        # print(f'Front: {self.front} Left: {self.left} Right: {self.right}')
        # print(f'Velocity: {self.linear} Angular Velocity : {self.angular}')
        # print(f'Prev State: {self.prevState} Current State : {self.state}')
        # print(f'Dir: {self.direction} ')
        if(msg.header.frame_id == "base_ir_front"):
            self.front = msg.range
        elif(msg.header.frame_id == "base_ir_left"):
            self.right = msg.range

        elif(msg.header.frame_id == "base_ir_right"):
            self.left = msg.range

        self.controller()

    def controller(self):
        # if(self.direction == 'north'):

        if(self.front < 2):
            self.linear = 0
            self.angular = 1.57
            print("FIRST")
            # rospy.sleep(1.5)
            # self.direction = 'south'

        elif(self.front == 2):

            if(self.left > 0.5 and self.left < 1.5):
                self.angular = 0

            elif(self.left <= 0.5):
                self.angular = 0.5

            elif(self.left >= 1.5):
                self.angular = -0.5

        if(self.front >= 2 and self.left >= 2):
            self.angular = -1.57
            self.linear = 0.1

        # elif(self.direction == 'south'):

        #     if(self.front < 2):
        #         self.linear = 0
        #         self.angular = 1.57
        #         self.state = 3
        #         # self.direction = 'north'
        #         print("FIRST")

        #     elif(self.front == 2):
        #         if(self.state == 3):
        #             self.state = 0
        #             self.angular = pi
        #             self.direction = 'north'

        #         if(self.right > 0.5 and self.right < 1.5):
        #             self.angular = 0

        #         elif(self.right <= 0.5):
        #             self.angular = 0.5

        #         elif(self.right >= 1.5):
        #             self.angular = -0.5

        #         elif(self.left < 2):
        #             self.angular = 1.57

        #     else:
        #         self.state = 1
        #         self.angular = 0
        #         print("Else")

        # Saturation Cases

        if(self.linear <= 0.2):
            self.linear = 0.2

        if(self.linear >= 1):
            self.linear = 1

        if(self.angular >= pi):
            self.angular = pi

        if(self.front == 2):
            self.linear = 1

        self.vel = Twist()
        self.vel.linear.x = -1*self.linear
        self.vel.angular.z = self.angular

        self.vel_pub.publish(self.vel)


if __name__ == '__main__':
    rospy.init_node('mapper_node')
    mapperInit = mapperClass()
    rospy.spin()
