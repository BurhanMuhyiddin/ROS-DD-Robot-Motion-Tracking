import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2

def callback(data):

    im = data.data
    r_v = im[0:-1:3]
    g_v = im[1:-1:3]
    b_v = im[2:-1:3]
    b_v = b_v + (im[-1],)

    r_v = np.asarray(r_v)
    g_v = np.asarray(g_v)
    b_v = np.asarray(b_v)

    r_v = r_v.reshape(32, 32)
    g_v = g_v.reshape(32, 32)
    b_v = b_v.reshape(32, 32)
    
    im = np.concatenate((r_v, g_v, b_v), 0)
    #im = im.reshape(32, 32, 3)
    #print(im.shape)
    cv2.imshow('im', im)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #print(r_v.shape)
    #print(g_v.shape)
    #print(b_v.shape)

    #cv2.imshow('im', im)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/camera_image", Float64MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
    cv2.destroyAllWindows()
