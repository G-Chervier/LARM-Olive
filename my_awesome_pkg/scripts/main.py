#!/usr/bin/python3
import random
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Brain:
    def __init__(self, verbose = True):
        self.commandPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd = Twist()
        self.verbose = verbose

        self.vitesse = 2

        self.isTurning = False
        self.turningCount = 0
        self.maxTurning = 30

        rospy.init_node('move', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.scanner)
        rospy.Timer(rospy.Duration(0.1), self.run, oneshot=False )
    
    def getROI(self, data):
        return data.ranges[355:365]

    def setAmountRotation(self):
        self.maxTurning = random.randint(30, 60)
        if(self.verbose):
            print(f"amount of turn : {self.maxTurning}")


    def shouldTurn(self, roi):
        return min(roi) < 2

    def handleTurning(self):
        if(self.isTurning):
            self.turningCount += 1
            self.cmd.angular.z = 1

        if(self.turningCount > self.maxTurning):
            self.isTurning = False
            self.cmd.angular.z = 0
            self.turningCount = 0

            if(self.verbose):
                print("stop turning")


    def scanner(self, data):
        roi = self.getROI(data)

        if(self.shouldTurn(roi) and not self.isTurning):
            self.isTurning = True
            self.setAmountRotation()
            if(self.verbose):
                print("start turning")

        self.handleTurning()


    def run(self, data):
        self.cmd.linear.x = self.vitesse
        if(self.isTurning):
            self.cmd.linear.x = 0
        self.commandPublisher.publish(self.cmd)


brain = Brain(False)

print("Start move.py")
rospy.spin()
