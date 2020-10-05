#!/usr/bin/env python

import rospy
from ddbot_msgs.srv import (CalculateKinematics, CalculateKinematicsResponse)
import math

class KinematicsCalculator(object):
    def __init__(self):
        self.WHEEL_RADIUS = 0.0975
        self.DISTANCE_BETWEEN_WHEELS = 0.3810
        self.ROBOT_VELOCITY = 0.10
        self.STEER_ANGLE = math.pi / 60.0

    def calculate_kinematics(self, req):
        velocity = req.vel
        steer_angle = req.steer_angle

        rightWheelVelocity = velocity + (self.DISTANCE_BETWEEN_WHEELS) * steer_angle
        leftWheelVelocity = velocity - (self.DISTANCE_BETWEEN_WHEELS) * steer_angle
        rightAngularVelocity = rightWheelVelocity / self.WHEEL_RADIUS
        leftAngularVelocity = leftWheelVelocity / self.WHEEL_RADIUS

        response = CalculateKinematicsResponse()
        response.rVelocity = rightAngularVelocity
        response.lVelocity = leftAngularVelocity

        return response

def main():
    rospy.init_node('kinematics_server')
    kinematics = KinematicsCalculator()
    calculate_kinematics = rospy.Service('calculate_kinematics', CalculateKinematics, kinematics.calculate_kinematics)
    rospy.spin()

if __name__ == '__main__':
    main()
