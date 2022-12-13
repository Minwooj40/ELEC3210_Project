#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped


bridge = CvBridge()
laser_scan_on = True

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
        
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([20, 100, 100])
    upper_bound = np.array([30, 255, 255])
    mask = cv2.inRange(image_hsv, lower_bound, upper_bound)

    mask_temp1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((10, 10)))
    mask_temp2 = cv2.morphologyEx(mask_temp1, cv2.MORPH_CLOSE, np.ones((20, 20)))
    cv2.imshow("masked image", mask_temp2)
    cv2.waitKey(1)

    _, contours, _ = cv2.findContours(mask_temp2, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    # if len(contours) == 0 or laser_scan_on:
    if len(contours) == 0:
        # print("2", x, y)
        return

    image_temp = image
    box_x, box_y, box_width, box_height = cv2.boundingRect(contours[0])
    cv2.rectangle(image_temp, (box_x, box_y), (box_x + box_width, box_y + box_height), (255, 0, 0), 2)
    cv2.imshow("boxed camera image", image_temp)
    cv2.waitKey(1)


    image_height, image_width = image.shape[:2]
    ball_center_x = box_x + (box_width / 2)
    ball_center_y = box_y + (box_height / 2)
    area_ratio = (float(box_height * box_width)) / (image_height * image_width)

    pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    if area_ratio:
        box_x2 = box_x + box_width
        box_y2 = box_y + box_height
        
        if box_x < 50:
            twist.angular.z = 0.5
        elif box_x2 > 450:
            twist.angular.z = -0.5
        else:
            twist.angular.z = float((image_height / 2) - ball_center_x) * 0.7 / (image_width / 2)
        
        if area_ratio < 0.05:
            twist.linear.x = 1.0
        elif area_ratio < 0.15:
            twist.linear.x = 0.8
        elif area_ratio > 0.25:
            twist.linear.x = -0.5
        else:
            twist.linear.x = 0.3
    else:
        return
        
    pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('visual_servoing')
    rospy.Subscriber('/vrep/laser_switch', Bool, auto_mode_callback)
    rospy.Subscriber('/vrep/image', Image, image_callback)
    rospy.spin()