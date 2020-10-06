#!/usr/bin/env python

# create a class to get client id(subscriber), angle(subscriber)
# and calculate kinematics. Not use server. write kinematics code
# inside this. Set joints of the robot

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
        self.STEER_ANGLE = math.pi / 15.0
        self.Kp = 0.4
        self.velocity = 0.05
        
        res, self.handle_right_wheel = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking)
        res, self.handle_left_wheel = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_blocking)
        res, self.handle_DDrobot = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx", sim.simx_opmode_blocking)
        res, self.handle_object = sim.simxGetObjectHandle(self.clientID, "Cuboid", sim.simx_opmode_blocking)

        rospy.Subscriber('/center', Float64, self.set_joint_angles)
        res, dist_DDrobot = sim.simxGetObjectPosition(self.clientID, self.handle_DDrobot, self.handle_object, sim.simx_opmode_streaming)

    def set_joint_angles(self, data):
        steer_angle = 0.0
        offset = data.data
        res, dist_DDrobot = sim.simxGetObjectPosition(self.clientID, self.handle_DDrobot, self.handle_object, sim.simx_opmode_buffer)

        #rospy.loginfo(offset)
        if math.sqrt(dist_DDrobot[0] * dist_DDrobot[0] + dist_DDrobot[1] * dist_DDrobot[1]) < 0.5:
            self.velocity = 0.0
            self.steer_angle = 0.0
        else:
            self.velocity = 0.05
        if abs(offset) < 0.5:
            steer_angle = 0.0
        elif offset == 999999999.0:
            self.velocity = 0.0
            steer_angle = math.pi/90
        else:
            k = abs(offset) / (offset) * (-1.0) # get the sign of the offset to control the robot (+: turn right, -: turn left)
                                                # multiplying by -1 is just for accomodating for the simulation
            steer_angle = k * self.Kp * self.STEER_ANGLE

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
