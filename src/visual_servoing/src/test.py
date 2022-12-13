#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist

bridge = CvBridge()

def auto_mode_callback(msg):
    global laser_scan_on
    laser_scan_on = msg.data

def image_callback(msg):
    image = None
    # recieve image from CvBridge and if fail print the error
    try:
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        image = cv2.flip(image, 1)
    except CvBridgeError as error:
        print(error)
    
    ### detecting the ball and its location from the image ###
    image_hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    # The RGB range for finding the yellow (255, 255, 0) ball is set here
    # but is a bit loose as the color of the ball varies due to light and curves
    # mask = cv2.inRange(image_hsv,np.array([30, 100, 100]),np.array([0, 255, 255]))
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(image_hsv, lower_yellow, upper_yellow)

    # For better detection mask out the ball using mophology transformation
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((10, 10)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((20, 20)))
    cv2.imshow("masked image",mask)

    mask_tmp = mask.copy()
    _, contours, _ =cv2.findContours(mask_tmp,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
    # If the yellow ball is not detected by the camera return
    if len(contours) == 0:
        return
    
    image_temp = image
    box_x, box_y, box_width, box_height = cv2.boundingRect(contours[0])
    cv2.rectangle(image_temp, (box_x, box_y), (box_x + box_width, box_y + box_height), (0, 0, 255), 1)
    cv2.imshow("boxed camera image", image_temp)

    # ball_center_x = box_x + (box_width / 2)
    # ball_center_y = box_y + (box_height / 2)
    # area_ratio = (float(box_height * box_width)) / (512 * 512)

    for c in contours:
        M = cv2.moments(c)
        cX = float(M["m10"]/M["m00"])
        cY = float(M["m01"]/M["m00"])
        rX = int(M["m10"]/M["m00"])
        rY = int(M["m01"]/M["m00"])
        radius = int(math.sqrt(cv2.contourArea(c)/math.pi))

    h,w = image.shape[:2]
    (ideal_X, ideal_Y) = (w/2, h-(20 + radius))
    verticle_diff = cY-ideal_Y
    angle_diff = cX-ideal_X
        
    pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
        
    twist = Twist()
    #linear
    if verticle_diff <= -50:
        twist.linear.x = 1.1
    elif (verticle_diff > -50) & (verticle_diff < 0):
        twist.linear.x = 0.5
    elif verticle_diff >= 20:
        twist.linear.x = -0.6
    elif (verticle_diff <20) & (verticle_diff > 5):
        twist.linear.x = -0.3
    else:
        twist.linear.x = 0    
    #angular
    if angle_diff >= 30:
        twist.angular.z = -1
    elif (angle_diff < 30) & (angle_diff > 10):
        twist.angular.z = -0.5
    elif angle_diff <= -30:
        twist.angular.z = 1
    elif (angle_diff > -30) & (angle_diff < -10):
        twist.angular.z = 0.5
    else:
        twist.angular.z = 0
    
    pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('visual_servoing')
    rospy.Subscriber('/vrep/laser_switch', Bool, auto_mode_callback)
    rospy.Subscriber('/vrep/image', Image, image_callback)
    rospy.spin()
