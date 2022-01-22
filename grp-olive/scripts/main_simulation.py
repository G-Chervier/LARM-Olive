#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray,Marker
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge
import tf
import math

CAMERA_ANGLE = 87.5

class Bottle: #Checks if a bottle is on the view of the camera and send a topic /bottle if true
    def __init__(self):
        self.tflistener = tf.TransformListener()
        self.sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.detection)
        self.sub2 = rospy.Subscriber("camera/depth/image_raw",Image,self.coords)
        self.pub = rospy.Publisher('bottle',MarkerArray, queue_size=1)
        self.pub2 = rospy.Publisher("bottle_in_base_footprint", PoseStamped, queue_size=1)
        self.sub3 = rospy.Subscriber("bottle_in_base_footprint",PoseStamped,self.convert)
        self.markerArray = MarkerArray() #List of the markers detected
        self.objx = 0 #declaration of X detected object in the frame
        self.objy = 0 #declaration of Y detected object in the frame
        self.detected = False #declaration of variable used if object detected
        self.countframes = 0 #count the frames an object has been detected
        self.allowdetection = True #allows detection

        #Camera angle = 87.5

    def pixtoangle(self, f,pix):
        ang = pix * CAMERA_ANGLE / f.shape[1] ##Calc angle from 0 to f.shape
        ang -= CAMERA_ANGLE/2 #Apply offset to set 0 in the middle
        # print("DBG : Angle = " + str(ang))
        # print("DBG : where x = "+ str(pix))
        return math.radians(-ang)

    def coords(self,img):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(img,desired_encoding="passthrough") #convert the image from topic sent to list of distances by pixel
        if self.detected and self.countframes>10:
            self.detected=False
            self.allowdetection=False
            
            if(math.isnan(frame[self.objy,self.objx])):
                self.allowdetection=True
            else:
                self.publish(self.pixtoangle(frame, self.objx),frame[self.objy,self.objx])

    def detection(self,img):
        bridge = CvBridge()     
        frame = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8') #convert the image from topic sent to readable image for opencv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        #Conert from BRG to HSV
        hsv = cv2.GaussianBlur(hsv,(7,3),1/9)               #Reduce noise
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)       #Get a gray image
        mask = cv2.inRange(hsv,(10,235,100),(20,255,130))  #sets the color (HSV) range to detect
        mask = cv2.dilate(mask,(3,3),iterations=1)
        mask = cv2.erode(mask,(3,3),iterations=1)
        #mask2 = cv2.inRange(hsv,(20,20,20),(self.i,self.i,self.i))     #To detect the black bottles (not working, too much black)
        detect = cv2.bitwise_and(gray,gray,mask=mask)   #get only the detected pixels
        elements=cv2.findContours(detect, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  #Find the contours of the objects detected
        color_infos = 255
        if len(elements) > 0: #if at least 1 elt detected 
            c=max(elements, key=cv2.contourArea) 
            ((x, y), rayon)=cv2.minEnclosingCircle(c)  #Get the position of object in the frame
            if rayon>15: #useful for decreasing the "noise" objects detection
                if self.allowdetection:
                    self.objx = int(x)
                    self.objy = int(y)
                    self.countframes +=1
                    self.detected=True
                # cv2.circle(frame, (int(x), int(y)), int(rayon), color_infos, 2)
                # cv2.circle(frame, (int(x), int(y)), 5, color_infos, 10)
                # cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), color_infos, 2)
                # cv2.putText(frame, "Bottle !", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_infos, 1, cv2.LINE_AA)
        else:
            self.detected = False
            self.allowdetection=True
            self.countframes = 0
        #cv2.circle(hsv, (self.objx, self.objy), 5, color_infos, 10)
        #cv2.imshow('frame', frame)
        #cv2.imshow('detected',detect)
        if cv2.waitKey(1)&0xFF==ord('q'):
            cv2.destroyAllWindows()

    def publish(self, angle,d):
        obj = PoseStamped()
        obj.header.frame_id="base_footprint"
        # print("DBG : d = " + str(d))
        obj.pose.position.y= d * math.sin(angle)
        obj.pose.position.x= d * math.cos(angle)
        obj.pose.position.z = 0
        obj.pose.orientation = Quaternion()
        # print("DBG : Obj")
        # print(obj)
        self.pub2.publish(obj)

    def convert(self, obj):
        # print("Got an object")
        globalpos = self.tflistener.transformPose('map',obj)
        marker= Marker( #declaration of a marker
                id =len(self.markerArray.markers)+1,
                type=Marker.CUBE,
                lifetime=rospy.Duration(0),
                scale=Vector3(0.1, 0.1, 0.1),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                pose = globalpos.pose)
        marker.header.frame_id='map'
        # print("DBG : Marker")
        # print(marker)
        self.markerArray.markers.append(marker)
        self.pub.publish(self.markerArray)

rospy.init_node('bottle_detector',anonymous=True)
myBottle = Bottle()
rospy.spin()