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

class detectionModel:
    def __init__(self):
        self.publisher = rospy.Publisher("image_marker", Marker, queue_size = 10)
        #queue_size is the size of the outgoing message queue used for asynchronous publishing.
        self.subscriber = rospy.Subscriber("/vrep/image", Image, self.callback) 
        self.bridge = CvBridge() #convert ROS image to OpenCV image
        self.pictures = [cv2.imread("/home/minwoo/ELEC3210_Project/src/image_detect/picture/" + name + ".jpg") for name in fileName]

#########################################################
        #Image features
        self.orb_cnt = self.square_cnt = [0] * len(self.pictures)
        self.orb = cv2.ORB_create()
        self.keyPoints, self.descriptors = [], []

        for i, picture in enumerate(self.pictures):
            imgSize = (400, 400)
            # picture = cv2.resize(picture, imgSize, interpolation = cv2.INTER_AREA)
            picture = cv2.flip(picture, 1)
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
            marker.text = fileName[i]
            marker.action = marker.ADD

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            ##coordinate
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 50.0
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
            if(dist[0][0] < 30.0):
                # print("Detect the image and inference the best fit", dist[0][1])
                return dist[0][1]
            else:
                # print("FALSE DETECTION")
                return -1
        


        # try:
        #     dist = []
        #     keyPoint = self.orb.detect(img, None)
        #     keyPoint, descriptor = self.orb.compute(img, keyPoint)

        #     #cv2.NORM_HAMMING
        #     for i, des in enumerate(self.descriptors):
        #         bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        #         matches = bf.match(des, descriptor)
        #         dist.append(matches.distance, i)

        #     dist = sorted(dist)
        #     print("Detect the image and inference the best fit", dist[0][1])
        #     return dist[0][1]
        # except:
        #     print("ERRRRRROROROROROROROROROROROROROR")
        #     return -1
        

    def marking(self, id, img):
        # print("Found the image and start to mark")## work
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, img_BW = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(img_BW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        ratio = float(w)/h

        if ratio > 1.1 and ratio < 0.5:
            self.square_cnt[id] = 0
        else:
            self.square_cnt[id] += 1

        if (self.square_cnt[id] >= 15):
            self.markers[id].pose.position.x = 1 / math.tan(math.pi / 8 * h / img.shape[1]) * 0.5
            self.markers[id].pose.position.y = (x + w/2) / img.shape[1]
            self.markers[id].pose.position.z = 3

            self.publisher.publish(self.markers[id])
            self.marked[id] = True
            #return


    def callback(self, img):
    
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        bestId = self.imageFit(img)
        if bestId == -1: return
        else:
            if(self.marked[bestId] == False):
                # print("Found image start marking")
                self.marking(bestId, img)
            else:
                a = 5
                # print("already marked")

def main():
    rospy.init_node("image_detect")
    node = detectionModel()
    rospy.spin()
    
if __name__ == "__main__":
    main()
