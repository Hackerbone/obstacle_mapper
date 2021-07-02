#!/usr/bin/env python
import rospy
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
        self.linear = 1
        self.angular = 0

        self.kpAngular = 0.8  # Proportional gain
        self.kiAngular = 0  # Integral gain
        self.kdAngular = 8  # Derivative gain
        self.dt = 0.16689300537109375
        self.errors = 0         # Integral value
        self.prevErr = 0        # Derivative value
        self.imax = 1           # Max integral value
        self.imin = -1          # Min integral value
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Setpoint distance from wall
        self.setpoint = 1

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
        print(arena_map)

    #     if(self.left == 2):
    #         if(self.poseY + 10 + self.left >= 20):
    #             arena_map[Math.floor(self.poseX + 10)
    #                       ][Math.floor(self.poseY + 10 - self.left)] = 3
    #         else:
    #             arena_map[Math.floor(self.poseX + 10)
    #                       ][Math.floor(self.poseY + 10 - self.left)] = 3

    #     if(self.right == 2):
    #         if(self.poseY + 10 + self.right >= 20):
    #             arena_map[Math.floor(self.poseX + 10)
    #                       ][Math.floor(self.poseY + 10 - self.right)] = 2
    #         else:
    #             arena_map[Math.floor(self.poseX + 10)
    #                       ][Math.floor(self.poseY + 10 - self.right)] = 1
    #     print(
    #         f'Left Calc : {Math.floor(self.poseY + 10 + self.left)} | {Math.floor(self.poseY + 10 - self.left)} ')
    #     print(
    #         f'Right Calc : {Math.floor(self.poseY + 10 + self.right)} | {Math.floor(self.poseY + 10 - self.right)} ')

    def subscriber(self, msg):
        print("\n\n")
        print(f'Front: {self.front} Left: {self.left} Right: {self.right}')
        print(f'Velocity: {self.linear} Angular Velocity : {self.angular}')
        print("\n\n")

        if(msg.header.frame_id == "base_ir_front"):
            self.front = msg.range
        elif(msg.header.frame_id == "base_ir_left"):
            self.right = msg.range

        elif(msg.header.frame_id == "base_ir_right"):
            self.left = msg.range

        self.calculateAngularVel()
        self.calculateLinearVel()

        self.controller()

    def calculateLinearVel(self):
        # Only proportional control
        # Based on left distance
        if(self.error < 1 or self.error > -1):
            self.linear = 1 - (0.6) * abs(self.error)

        # Based on front distance
        if(self.front < 3.3):
            self.linear = 1 - (0.33) * abs(2-self.front)

    def calculateAngularVel(self):
        # Error term for angular PID
        self.error = (self.setpoint - self.left)

        # Proportional term
        self.pAngular = self.kpAngular * self.error

        # Integral term
        self.errors += self.error
        if(self.errors > self.imax):
            self.errors = self.imax
        elif(self.errors < self.imin):
            self.errors = self.imin
        self.iAngular = self.kiAngular * self.errors

        # Derivative Term
        self.dAngular = self.kdAngular * (self.error - self.prevErr) / self.dt
        self.prevErr = self.error

        # Total response
        self.angular = self.pAngular + self.iAngular + self.dAngular

        # Saturating the angular responses
        if(self.angular > pi/2):
            self.angular = pi/2
        elif(self.angular < -1 * pi/2):
            self.angular = -1*pi/2

    def controller(self):

        if self.left > 4 and self.front > 4:
            self.angularResponse = -1 * pi/8
            self.linear = 0.5
        # # Front obstacle avoidance
        if self.front < 3.3:
            self.angular = 1.8

        # Linear velocity saturation
        if self.linear > 1:
            self.linear = 1
        if self.linear < 0.1:
            self.linear = 0.1

        self.vel = Twist()
        self.vel.linear.x = -1*self.linear
        self.vel.angular.z = self.angular

        self.vel_pub.publish(self.vel)


if __name__ == '__main__':
    rospy.init_node('mapper_node')
    mapperInit = mapperClass()
    rospy.spin()
