#! /usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge , CvBridgeError
import sys
from canny import *

bridge = CvBridge()
prop = 0
diff = 0
integral = 0
kP = 0.0009
kD = 0.018
kI = 0.0000009
integralActivatingZone = 10
last_error = 0
pub = rospy.Publisher("/cmd_vel" , Twist )
tot_error = 0
frame = 0
img_count = 582

def posneg(error):
    if error >= 0:
        return 'pos'
    else:
        return 'neg'

def steeringerror(error):
    error = abs(error)
    if(error/10 == 0):
        return '0'+error
    else:
        return str(error)

def give_steering_angle(error):
    global kP , kD , kI , last_error , integralActivatingZone , prop , diff , integral , pub , tot_error

    motor_vals = Twist()

    if( error < integralActivatingZone and error != 0):
        tot_error += error
    else:
        tot_error = 0
    
    if error == 0:
        diff = 0
        prop = 0
    

    prop += error                 *kP
    diff += (error - last_error)  *kD
    integral += tot_error         *kI

    last_error = error

    steering_val_to_motors = prop+diff+integral
    motor_vals.linear.x = 1
    motor_vals.linear.y = 0
    motor_vals.linear.z = 0

    motor_vals.angular.x = 0
    motor_vals.angular.y = 0
    motor_vals.angular.z = -steering_val_to_motors

    pub.publish(motor_vals)
    rospy.sleep(0.005)

def callback(ros_image):
    global bridge , frame , img_count
    frame += 1
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    gray = cv2.cvtColor(cv_image , cv2.COLOR_BGR2GRAY)
    canny = apply_canny(gray)

    red_edge = detect_red(cv_image)
    fin_image = cv2.bitwise_or(red_edge , canny)

    ROI = get_reqd_region(fin_image)
    
    lines = cv2.HoughLinesP(ROI , 1, np.pi/180 , 150 , np.array([]) , minLineLength=40 , maxLineGap=50)
    averaged_lines = average_slope_intercept(cv_image , lines)
    line_image = display_lines(cv_image , averaged_lines)
    combo_image = cv2.addWeighted(cv_image , 0.8 , line_image , 1 , 1)
    
    if len(averaged_lines) > 0:
        steering_error = compute_steering_angles(cv_image , averaged_lines)
        correction_image = show_steering_corection(combo_image , steering_error)
        #Draw the line which shows how much the bot is deviated from current heading 
        combo_image = cv2.addWeighted(combo_image , 0.9 , correction_image , 1 , 1)
        give_steering_angle(steering_error)

        if (frame%3 == 0):
            img_count+=1
            cv2.imwrite('dataset/image_'+str(img_count)+'_'+posneg(steering_error)+'_'+steeringerror(steering_error)+'.jpg' , cv_image)
    
    #My heading
    cv2.line(combo_image , (600 , cv_image.shape[0]) , (600 , cv_image.shape[0]-300) , (0 , 255 , 0) , 5 )

    if cv2.waitKey(2)&0xff == ord('w'): #telling the program to wait
        cv2.waitKey(0)

    cv2.imshow("combo_image" , combo_image)
    #cv2.imshow("original" , cv_image)
    #cv2.imshow("ROI" , ROI)
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