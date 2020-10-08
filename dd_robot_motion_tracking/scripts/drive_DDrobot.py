#!/usr/bin/env python

import sim
import rospy
from std_msgs.msg import Float64
import math
import time

class Drive_Robot(object):
    def __init__(self, clientID):
        # declare some robot parameters
        self.clientID = clientID
        self.WHEEL_RADIUS = 0.0975
        self.DISTANCE_BETWEEN_WHEELS = 0.3810
        self.dist_from_Bill = 4.0
        
        # declare PID gains for the angle control
        self.Kp_a = 0.0015  # Proportional gain
        self.Kd_a = 0.00002 # Derivative gain
        self.Ki_a = 0.00035 # Integral gain
        self.integral_term = 0.0

        # declare P gain for the velocity control
        self.Kp_v = 0.2

        self.velocity = 0.0
        self.steer_angle = 0.0
        
        self.current_time = time.time()
        self.last_time = self.current_time
        self.last_offset = 0.0
        
        res, self.handle_right_wheel = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking)
        res, self.handle_left_wheel = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_blocking)
        res, self.handle_DDrobot = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx", sim.simx_opmode_blocking)
        res, self.handle_distance = sim.simxGetDistanceHandle(self.clientID, "Distance", sim.simx_opmode_blocking)
        
        res, self.distance = sim.simxReadDistance(self.clientID, self.handle_distance, sim.simx_opmode_streaming)

        rospy.Subscriber('/offset', Float64, self.set_joint_angles)

    def set_joint_angles(self, data):
        offset = data.data
        res, self.distance = sim.simxReadDistance(self.clientID, self.handle_distance, sim.simx_opmode_buffer) # read distance between
                                                                                                               # robot and Bill 
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_offset = offset - self.last_offset

        self.velocity = self.Kp_v * (self.dist_from_Bill - self.distance) * (-1.0) # try to keep robot a bit away from Bill, so can process him in camera

        # Calculate the PID terms and control the angle properly, so robot can try to keep Bill in the center of camera
        self.integral_term = self.integral_term + offset * delta_time
        self.steer_angle = ( self.Kp_a * offset + self.Kd_a * (delta_offset / delta_time) + self.Ki_a * self.integral_term) * (-1.0)
        
        # Calculate the kinematics for left and right wheels
        rightWheelVelocity = self.velocity + (self.DISTANCE_BETWEEN_WHEELS) * self.steer_angle
        leftWheelVelocity = self.velocity - (self.DISTANCE_BETWEEN_WHEELS) * self.steer_angle
        rightAngularVelocity = rightWheelVelocity / self.WHEEL_RADIUS
        leftAngularVelocity = leftWheelVelocity / self.WHEEL_RADIUS
        
        # Send the joint values to the joints
        sim.simxSetJointTargetVelocity(self.clientID, self.handle_right_wheel, rightAngularVelocity, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.clientID, self.handle_left_wheel, leftAngularVelocity, sim.simx_opmode_streaming)
        
        # update variables
        self.last_time = self.current_time
        self.last_offset = offset

def main():
    rospy.init_node('drive_node')
    clientID = sim.simxStart('127.0.0.1', 29999, True, True, 5000, 5)
    if clientID != -1:
        dr = Drive_Robot(clientID)
    rospy.spin()

if __name__ == '__main__':
    main()
