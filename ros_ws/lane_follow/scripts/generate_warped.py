#! /usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
import sys
from utils import *

bridge = CvBridge()
maxWidth = 1075
maxHeight = 645
points = np.array([[540 , 150] , [756 , 150] , [1135 , 795] , [60 , 795]] , dtype="float32")

def get_reqd_region(image):
    global maxHeight , maxWidth , points
    height = image.shape[0]
    dst = np.array([
		[0, 0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]], dtype = "float32")
    M = cv2.getPerspectiveTransform(points , dst)
    warped = cv2.warpPerspective(image , M , (maxWidth , maxHeight))
    return warped

def callback(ros_image):
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    canny = apply_canny(cv_image)
    ROI = get_reqd_region(canny)
    cv2.imshow("acquired lane" , cv_image)
    cv2.imshow("canny" , canny)
    cv2.imshow("ROI" , ROI)
    cv2.waitKey(3)


def main():
    rospy.init_node("get_img")
    img_sub = rospy.Subscriber("/botcam/camera1/image_raw" , Image , callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('ShutDown!...')

    cv2.destroyAllWindows()

if __name__=='__main__':
    main()