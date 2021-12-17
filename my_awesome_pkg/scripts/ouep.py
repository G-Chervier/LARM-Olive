#!/usr/bin/python3
import  rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Brain:
    def __init__(self):
        rospy.init_node('move', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.scanner)
        self.commandPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd = Twist()

        self.vitesse = 1

        self.isTurning = False
        self.turningCount = 0
        self.maxTurning = 1000

    def getROI(self, data):
        return data.ranges[355:365]

    def shouldTurn(self, roi):
        return min(roi) < 2

    def handleTurning(self):
        if(self.isTurning):
            self.turningCount += 1
            self.cmd.angular.z = 1

        if(self.turningCount > self.maxTurning):
            self.isTurning = False
            self.turningCount = 0


    def scanner(self, data):
        roi = self.getROI(data)

        if(self.shouldTurn(roi)):
            self.isTurning = True

        self.handleTurning()


    def run(self):
        self.cmd.linear.x = self.vitesse
        if(self.isTurning):
            self.cmd.linear.x = 0
        self.commandPublisher.publish(self.cmd)


brain = Brain()
rospy.Timer(rospy.Duration(0.1), brain.run(), oneshot=False )
