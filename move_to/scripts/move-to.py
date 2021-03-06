#!/usr/bin/env python3
from threading import local
import rospy
import tf
from geometry_msgs.msg import *



class myNode:
    def __init__(self):
        rospy.init_node('move_to', anonymous=True)
        self.tfListener = tf.TransformListener()
        self.sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
        self.pub = rospy.Publisher(
            '/cmd_vel_mux/input/navi',
            Twist, queue_size=10
        )
        rospy.Timer( rospy.Duration(0.1), self.callback2, oneshot=False )
    
    def callback(self, goal):
        rospy.loginfo("I Got a goal : ")
        print(goal)
        self.local_goal= self.tfListener.transformPose("/base_footprint", goal)
        print(self.local_goal)

    def callback2(self,data):
        #Do some work
        cmd= Twist()
        cmd.linear.x= 0.1
        cmd.angular.z = self.local_goal.pose.orientation.z
        self.pub.publish(cmd)
        

myNode()
rospy.spin()