#!/usr/bin/env python

import sim
import rospy
from std_msgs.msg import Float64
import math

class Drive_Robot(object):
    def __init__(self, clientID):
        self.clientID = clientID
        self.WHEEL_RADIUS = 0.0975
        self.DISTANCE_BETWEEN_WHEELS = 0.3810
        self.ROBOT_VELOCITY = 0.10
        self.STEER_ANGLE = math.pi / 30.0
        self.Kp_a = 0.0015
        self.Kp_v = 0.04
        self.velocity = 0.2
        
        res, self.handle_right_wheel = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking)
        res, self.handle_left_wheel = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_blocking)
        res, self.handle_DDrobot = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx", sim.simx_opmode_blocking)
        res, self.handle_person = sim.simxGetObjectHandle(self.clientID, "Bill", sim.simx_opmode_blocking)

        rospy.Subscriber('/offset', Float64, self.set_joint_angles)
        res, self.dist_DDrobot = sim.simxGetObjectPosition(self.clientID, self.handle_DDrobot, self.handle_person, sim.simx_opmode_streaming)

    def set_joint_angles(self, data):
        steer_angle = 0.0
        offset = data.data
        res, dist_DDrobot = sim.simxGetObjectPosition(self.clientID, self.handle_DDrobot, self.handle_person, sim.simx_opmode_buffer)
        
        dist = math.sqrt(dist_DDrobot[0] * dist_DDrobot[0] + dist_DDrobot[1] * dist_DDrobot[1]) 
        if dist <= 3.0:
            #self.velocity = 0.0
            self.velocity = self.Kp_v * dist * -1.0
        elif dist >= 3.5:
            self.velocity = self.Kp_v * dist

        #if abs(offset) <= 0.1:
        #    steer_angle = 0.0
        #elif offset == 999999999.0:
        #    self.velocity = 0.0
        #    steer_angle = math.pi/90
        #elif abs(offset) >= 0.5:
        #    k = abs(offset) / (offset) * (-1.0) # get the sign of the offset to control the robot (+: turn right, -: turn left)
                                                # multiplying by -1 is just for accomodating for the simulation
        #    steer_angle = k * self.Kp_a * self.STEER_ANGLE

        steer_angle = self.Kp_a * offset * (-1.0)

        rightWheelVelocity = self.velocity + (self.DISTANCE_BETWEEN_WHEELS) * steer_angle
        leftWheelVelocity = self.velocity - (self.DISTANCE_BETWEEN_WHEELS) * steer_angle
        rightAngularVelocity = rightWheelVelocity / self.WHEEL_RADIUS
        leftAngularVelocity = leftWheelVelocity / self.WHEEL_RADIUS
        
        #rospy.loginfo(rightAngularVelocity)
        #rospy.loginfo(leftAngularVelocity)

        sim.simxSetJointTargetVelocity(self.clientID, self.handle_right_wheel, rightAngularVelocity, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.clientID, self.handle_left_wheel, leftAngularVelocity, sim.simx_opmode_streaming)


def main():
    rospy.init_node('drive_node')
    clientID = sim.simxStart('127.0.0.1', 29999, True, True, 5000, 5)
    if clientID != -1:
        dr = Drive_Robot(clientID)
    rospy.spin()

if __name__ == '__main__':
    main()
