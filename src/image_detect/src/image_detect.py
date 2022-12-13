#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
import math
import time
import cv2

fileName = ["pic001", "pic002", "pic003", "pic004", "pic005"]
faceName = ["Obama", "Avril", "Cheung", "Rings", "Levi" ]

class detectionModel:
    def __init__(self):
        self.publisher = rospy.Publisher("image_marker", Marker, queue_size = 10)
        #queue_size is the size of the outgoing message queue used for asynchronous publishing.
        self.subscriber = rospy.Subscriber("/vrep/image", Image, self.callback) 
        self.bridge = CvBridge() #convert ROS image to OpenCV image
        self.pictures = [cv2.imread("/home/sieun/catkin_ws/src/image_detect/picture/" + name + ".jpg") for name in fileName]

#########################################################
        #Image features
        self.detectionCnt = [0] * len(self.pictures)
        self.orb = cv2.ORB_create()
        self.keyPoints, self.descriptors = [], []

        for i, picture in enumerate(self.pictures):
            imgSize = (400, 400)
            # picture = cv2.resize(picture, imgSize, interpolation = cv2.INTER_AREA)
            if(i == 0 or i == 1 or i == 3): picture = cv2.flip(picture, 1)
            keyPoint, descriptor = self.orb.detectAndCompute(picture, None)
            self.keyPoints.append(keyPoint) 
            self.descriptors.append(descriptor)

########################################################
        #Mark status
        self.marked = [False] * len(self.pictures)
        self.markers = []
        
        for i in range(len(fileName)):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.ns = "picture"
            marker.type = marker.TEXT_VIEW_FACING
            marker.text = faceName[i]
            marker.action = marker.ADD

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            ##coordinate
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.id = i

            marker.lifetime = rospy.Duration(1000000)
            self.markers.append(marker)


    def imageFit(self, img):
        dist = []
        keyPoint = self.orb.detect(img, None)
        keyPoint, descriptor = self.orb.compute(img, keyPoint)

        #cv2.NORM_HAMMING
        if descriptor is None:
            return -1

        else:
            for i, des in enumerate(self.descriptors):
                bf = cv2.BFMatcher(cv2.NORM_HAMMING)
                matches = bf.match(des, descriptor)
                matches = sorted(matches, key = lambda x:x.distance)
                dist.append([matches[0].distance, i])

            dist = sorted(dist)
            #print(dist[0][0], dist[0][1])
            if(dist[0][0] < 17.0):
                # print("Detect the image and inference the best fit", dist[0][1])
                return dist[0][1]
            else:
                # print("FALSE DETECTION")
                return -1
        

    def marking(self, id, img):
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, grayImg = cv2.threshold(grayImg, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        _, grads, _ = cv2.findContours(grayImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        x, y, w, h = cv2.boundingRect(max(grads, key = cv2.contourArea))


        ###### Ratio Tests
        ratio = float(w)/h
        if ratio >= 0.5 and ratio <= 1.1:
            self.detectionCnt[id] += 1
        else:
            self.detectionCnt[id] = 0

        if (self.detectionCnt[id] >= 15):
            self.markers[id].pose.position.x = 1 / math.tan(math.pi / 8 * h / img.shape[1]) * 0.5
            self.markers[id].pose.position.y = (x + w/2) / img.shape[1]
            self.markers[id].pose.position.z = 0
            self.publisher.publish(self.markers[id])
            self.marked[id] = True

    def callback(self, img):
    
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        bestId = self.imageFit(img)
        if bestId == -1: return
        else:
            if(self.marked[bestId] == False):
                # print("Found image start marking")
                self.marking(bestId, img)

def main():
    rospy.init_node("image_detect")
    node = detectionModel()
    rospy.spin()
    
if __name__ == "__main__":
    main()
