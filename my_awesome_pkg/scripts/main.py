#!/usr/bin/python3
import  rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Brain:
    def __init__(self, verbose = True):
        self.commandPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd = Twist()
        self.verbose = verbose  

        self.vitesse = 1

        self.isTurning = False
        self.turningCount = 0
        self.maxTurning = 30

        rospy.init_node('move', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.scanner)
        rospy.Timer(rospy.Duration(0.1), self.run, oneshot=False )

    def getROI(self, data):
        return data.ranges[355:365]

    def shouldTurn(self, roi):
        return min(roi) < 0.4

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
            if(self.verbose):
                print("start turning")

        self.handleTurning()


    def run(self, data):
        self.cmd.linear.x = self.vitesse
        if(self.isTurning):
            self.cmd.linear.x = 0
        self.commandPublisher.publish(self.cmd)


brain = Brain()

print("Start move.py")
rospy.spin()
"""
#!/usr/bin/python3
import random 
import math, rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Initialize ROS::node
rospy.init_node('move', anonymous=True)
INITIAL_SPEED = 1
MAX_SPEED = 5

cmd = Twist()
vitesse = INITIAL_SPEED
acceleration = 0.1
isTurning = False
turningCount = 0
amountOfTurn = 0

commandPublisher = rospy.Publisher(
    '/cmd_vel',
    Twist, queue_size=10
)

# Publish velocity commandes:
def move_command(data):
    global vitesse, cmd
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd.linear.x= vitesse
    commandPublisher.publish(cmd)
# call the move_command at a regular frequency:
rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )

def callB(data):
    global vitesse, acceleration,INITIAL_SPEED, MAX_SPEED, isTurning, turningCount, amountOfTurn, cmd
    sample = data.ranges[355:365]

    if min(sample) < 6:
        if vitesse > 1:
            vitesse -= acceleration

    if turningCount > amountOfTurn:
        isTurning = False
        turningCount = 0

    if isTurning:
        turningCount += 1
        cmd.angular.z= 1.5 #add x, y, z angular 


    if min(sample) < 2 and not isTurning:
        amountOfTurn = 1000
        vitesse = INITIAL_SPEED
        isTurning = True

    if vitesse < MAX_SPEED:
        vitesse += acceleration


rospy.Subscriber("/scan", LaserScan, callB )

# spin() enter the program in a infinite loop
print("Start move.py")
rospy.spin()

"""