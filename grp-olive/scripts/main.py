#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image,LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge
import os
import numpy as np

CAMERA_ANGLE = 87.5

class Bottle: #Checks if a bottle is on the view of the camera and send a topic /bottle if true
    def __init__(self):
        self.odom = rospy.Subscriber("odom",Odometry,self.getPose)
        self.scan = rospy.Subscriber("scan",LaserScan,self.callbackLaser)
        self.sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.detection)
        self.sub2 = rospy.Subscriber("camera/depth/image_rect_raw",Image,self.coords)
        self.pub = rospy.Publisher('bottle',Marker, queue_size=5)
        #dirname = os.path.dirname(__file__)
        #filename = os.path.join(dirname, 'cascade.xml')
        #filename = "/home/gaetan/visionPython/classifier_training/ws/data/cascade.xml"
        #self.classifier = cv2.CascadeClassifier(filename)
        self.marker= Marker(
                type=Marker.CUBE,
                lifetime=rospy.Duration(0),
                scale=Vector3(0.1, 0.1, 0.1),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self.i = 97
        self.objx = 0
        self.objy = 0

    def souris(self, event, x, y, flags, param):
        if event==cv2.EVENT_LBUTTONDOWN:
            self.i+=1
        if event== cv2.EVENT_RBUTTONDOWN:
            self.i-=1

        #Camera angle = 87.5
        #voir si c'est vraiment une bouteille (rester sur image >10frames)

    def coords(self,img):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(img,desired_encoding="passthrough") #convert the image from topic sent to readable image for opencv
        print(frame[self.objy,self.objx])

    def detection(self,img):
        bridge = CvBridge()     
        frame = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8') #convert the image from topic sent to readable image for opencv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        #Conert from BRG to HSV
        hsv = cv2.GaussianBlur(hsv,(7,3),1/9)               #Reduce noise
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)       #Get a gray image
        mask = cv2.inRange(hsv,(0,97*255/100,60*255/100),(40,255,255))  #sets the color (HSV) range to detect
        mask = cv2.dilate(mask,(3,3),iterations=1)
        mask = cv2.erode(mask,(3,3),iterations=1)
        #mask2 = cv2.inRange(hsv,(20,20,20),(self.i,self.i,self.i))     #To detect the black bottles (not working, too much black)
        detect = cv2.bitwise_and(gray,gray,mask=mask)   #get only the detected pixels
        
        elements=cv2.findContours(detect, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  #Find the contours of the objects detected
        if len(elements) > 0:
            color_infos = 255
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)  #Get the position of object in the frame
            if rayon>15:
                self.objx = int(x)
                self.objy = int(y)
                cv2.circle(detect, (int(x), int(y)), int(rayon), color_infos, 2)
                cv2.circle(detect, (int(x), int(y)), 5, color_infos, 10)
                cv2.line(detect, (int(x), int(y)), (int(x)+150, int(y)), color_infos, 2)
                cv2.putText(detect, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_infos, 1, cv2.LINE_AA)
        #cv2.putText(hsv, str(self.i), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, (255,0,0), 1, cv2.LINE_AA)
        cv2.imshow('frame', hsv)
        cv2.imshow('detected',detect)
        #cv2.setMouseCallback('detected', self.souris)
        if cv2.waitKey(1)&0xFF==ord('q'):
            cv2.destroyAllWindows()

    def getPose(self,data):  #Pose of the robot in /odom
        self.marker.pose = data.pose.pose

    def publish(self):
        self.marker.header.frame_id='map'
        self.pub.publish(self.marker)

    def callbackLaser(self,LaserData):
        a=0
rospy.init_node('bottle_detector',anonymous=True)
myBottle = Bottle()
rospy.spin()