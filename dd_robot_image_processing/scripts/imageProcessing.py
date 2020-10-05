#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int8
import sim
import cv2
import numpy as np
import matplotlib.pyplot as mlp
import math

def image_processor(clientID):
    pub = rospy.Publisher('center', Float64, queue_size=10)
    rospy.init_node('image_processor_node')
    rate = rospy.Rate(10)
    res, handle_cam = sim.simxGetObjectHandle(clientID, "cam", sim.simx_opmode_blocking)
    while not rospy.is_shutdown():
        err, resolution, frame = sim.simxGetVisionSensorImage(clientID, handle_cam, 0, sim.simx_opmode_streaming)
        if (sim.simxGetConnectionId(clientID) != -1):
            err, resolution, frame = sim.simxGetVisionSensorImage(clientID, handle_cam, 0, sim.simx_opmode_buffer)
            frame = frame[::-1] # original was bgr, I converted to rgb
            if err == sim.simx_return_ok:
                img = np.array(frame, dtype=np.uint8)
                img.resize([resolution[0], resolution[1], 3])
                
                # detect red cube
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                l_b = np.array([0, 100, 100])
                u_b = np.array([10, 255, 255])

                mask = cv2.inRange(hsv, l_b, u_b)

                contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                if len(contours) != 0:
                    (x, y, w, h) = cv2.boundingRect(contours[0])
                    x_medium = (2.0 * x + w) / 2.0 # find the center of the object
                    object_center = (resolution[0]) / 2.0 - x_medium # calculate offset between object's center and the frame center
                    pub.publish(object_center)
                    #rospy.loginfo(res)

                    #cv2.line(img, (int(x_medium), 0), (int(x_medium), resolution[0]), (0, 0, 255), 1)
                    rate.sleep()

                #cv2.imshow('image', img)
                #if cv2.waitKey(1) & 0xFF == ord('q'):
                    #break
        else:
            print("Failed to connect to the remote API server...")
            sim.simxFinish(clientID)

if __name__ == '__main__':
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID != -1:
        image_processor(clientID)
    cv2.destroyAllWindows()
