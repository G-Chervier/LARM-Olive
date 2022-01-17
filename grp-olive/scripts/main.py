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

class Bottle: #Checks if a bottle is on the view of the camera and send a topic /bottle if true
    def __init__(self):
        self.odom = rospy.Subscriber("odom",Odometry,self.getPose)
        self.scan = rospy.Subscriber("scan",LaserScan,self.callbackLaser)
        self.sub = rospy.Subscriber("camera/color/image_raw",Image,self.detection)
        self.pub = rospy.Publisher('bottle',Marker, queue_size=5)
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, 'cascade.xml')
        filename = "/home/gaetan/visionPython/classifier_training/ws/data/cascade.xml"
        self.classifier = cv2.CascadeClassifier(filename)
        self.marker= Marker(
                type=Marker.CUBE,
                lifetime=rospy.Duration(0),
                scale=Vector3(0.1, 0.1, 0.1),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))

    def detection(self,img):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8') #convert the image from topic sent to readable image for opencv
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        objs = self.classifier.detectMultiScale(frame, 1.1,3) #detection of the objects

        for (x, y, w, h) in objs:
            print('I detect one')
            cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0), 2)
            self.publish()
        #cv2.imshow('bot view', gray)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1)&0xFF==ord('q'):
            cv2.destroyAllWindows()


    def getPose(self,data):
        self.marker.pose = data.pose.pose

    def publish(self):
        self.marker.header.frame_id='map'
        self.pub.publish(self.marker)

    def callbackLaser(self,LaserData):
        a=0
rospy.init_node('bottle_detector',anonymous=True)
myBottle = Bottle()
rospy.spin()