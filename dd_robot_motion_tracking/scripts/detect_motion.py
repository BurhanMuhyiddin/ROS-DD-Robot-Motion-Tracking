#!/usr/bin/env python

import rospy
import sim
import cv2
import numpy as np
import math
from std_msgs.msg import Float64

class MotionDetector(object):
    def __init__(self, clientID):
        self.clientID = clientID
        self.pub = rospy.Publisher('offset', Float64, queue_size=10)
        rospy.init_node('motion_tracker_node')
        self.rate = rospy.Rate(10)

        self.offset = 0.0

        res, self.handle_cam = sim.simxGetObjectHandle(self.clientID, "cam", sim.simx_opmode_blocking)
        
        self.track_motion()

    def track_motion(self):
        res, resolution, frame1 = sim.simxGetVisionSensorImage(self.clientID, self.handle_cam, 0, sim.simx_opmode_streaming)
        
        # wait until frame is read correctly
        while res != sim.simx_return_ok: 
            res, resolution, frame1 = sim.simxGetVisionSensorImage(self.clientID, self.handle_cam, 0, sim.simx_opmode_buffer)
        res, resolution, frame2 = sim.simxGetVisionSensorImage(self.clientID, self.handle_cam, 0, sim.simx_opmode_buffer)

        # convert from BGR to RGB, also the position of the image
        frame1 = frame1[::-1]
        frame1 = np.array(frame1, dtype=np.uint8)
        frame1.resize([resolution[0], resolution[1], 3])

        frame2 = frame2[::-1]
        frame2 = np.array(frame2, dtype=np.uint8)
        frame2.resize([resolution[0], resolution[1], 3])

        # Process the video and detect motion by subtracting subsequent frames
        while not rospy.is_shutdown():
            if sim.simxGetConnectionId(self.clientID) != -1:
                diff = cv2.absdiff(frame1, frame2)
                gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (5,5), 0)
                _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
                dilated = cv2.dilate(thresh, None, iterations=3)
                contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                counter = 0
                centerX = 0

                for contour in contours:
                    (x, y, w, h) = cv2.boundingRect(contour)
                    if w >= h or cv2.contourArea(contour) < 1000: # because human's height is greater than width
                        continue
                    counter = counter + 1
                    centerX = centerX + (2.0 * x + w) / 2.0
                    #cv2.rectangle(frame1, (x, y), (x+w, y+h), (0, 255, 0), 1)
                
                if counter != 0:
                    centerX = centerX / counter
                    self.offset = (resolution[0] / 2.0) - centerX
                else:
                    self.offset = 0.0

                self.pub.publish(self.offset) # publish the offset from the center

                cv2.imshow("camera", frame1)
                
                # update variables
                frame1 = frame2
                res, resolution, frame2 = sim.simxGetVisionSensorImage(self.clientID, self.handle_cam, 0, sim.simx_opmode_buffer)
                
                frame2 = frame2[::-1]
                frame2 = np.array(frame2, dtype=np.uint8)
                frame2.resize([resolution[0], resolution[1], 3])

                if cv2.waitKey(40) == 27:
                    break
            else:
                print("Failed to connect to the remote API server...")
                sim.simxFinish(self.clientID)


if __name__ == '__main__':
    sim.simxFinish(-1) # close if there are any other connections
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID != -1:
        MotionDetector(clientID)
        cv2.destroyAllWindows()
    else:
        print("Connection couldn't be established...")



