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
    err, resolution, frame = sim.simxGetVisionSensorImage(clientID, handle_cam, 0, sim.simx_opmode_streaming)
    print(err)

if __name__ == '__main__':
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 29999, True, True, 5000, 5)
    if clientID != -1:
        image_processor(clientID)
    cv2.destroyAllWindows()
