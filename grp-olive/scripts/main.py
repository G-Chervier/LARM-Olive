import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import os

class MyNode:
    def __init__(self):
        self.sub = rospy.Subscriber("camera/color/image_raw",Image,self.detection)
        self.odom = rospy.Subscriber("/odom",PoseStamped,self.getPose)
        self.pub = rospy.Publisher(
            '/bottle',
            Marker, queue_size=10
        )
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, 'cascade.xml')
        self.classifier = cv2.CascadeClassifier('cascade.xml')

    def detection(self,img):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.classifier.detectMultiScale(frame, 1.5, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0), 2)
            roiGray = gray[y:y+h, x:x+w]
            roiColor = frame[y:y+h, x:x+w]
        #cv2.imshow('bot view', gray)
        #cv2.imshow('frame', frame)
        if cv2.waitKey(1)&0xFF==ord('q'):
            cv2.destroyAllWindows()
    def publish(self):
        cmd = Marker()
        cmd.pose = self.pose
        self.pub.publish(cmd)

    def getPose(self,data):
        self.pose = data.pose

rospy.init_node('bottle_detector',anonymous=True)
myNode = MyNode()
rospy.spin()