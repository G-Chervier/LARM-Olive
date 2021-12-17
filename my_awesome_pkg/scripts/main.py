#!/usr/bin/python3
import random 
import math, rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Initialize ROS::node
rospy.init_node('move', anonymous=True)
INITIAL_SPEED = 1
MAX_SPEED = 5

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
    global vitesse
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()
    cmd.linear.x= vitesse
    commandPublisher.publish(cmd)
# call the move_command at a regular frequency:
rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )

def callB(data):
    global vitesse, acceleration,INITIAL_SPEED, MAX_SPEED, isTurning, turningCount, amountOfTurn
    sample = data.ranges[355:365]

    if min(sample) < 6:
        if vitesse > 1:
            vitesse -= acceleration

    if turningCount > amountOfTurn:
        isTurning = False
        turningCount = 0

    if isTurning:
        turningCount += 1
        cmd= Twist()
        cmd.angular.z= 1.5 #add x, y, z angular 
        commandPublisher.publish(cmd)


    if min(sample) < 2:
        amountOfTurn = random.randint(1, 10)
        vitesse = INITIAL_SPEED
        isTurning = True

    if vitesse < MAX_SPEED:
        vitesse += acceleration


rospy.Subscriber("/scan", LaserScan, callB )

# spin() enter the program in a infinite loop
print("Start move.py")
rospy.spin()
