#!/usr/bin/python3
import rospy
from math import radians,cos,sin
from random import randint
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

DIST_LASER_ROBOT = 0.5
FRONT_ANGLE = 90
isTurningr = False
isTurningl = False
countTurning = 0
# Initialize ROS::node
rospy.init_node('move', anonymous=True)

commandPublisher = rospy.Publisher(
    '/mobile_base/commands/velocity',
    Twist, queue_size=10
)
vitessex = 0
vitessez = 0
# Publish velocity commandes:
def move_command():
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()
    cmd.linear.x= vitessex
    cmd.angular.z = vitessez
    commandPublisher.publish(cmd)

def turn_left():
    global vitessex, vitessez, isTurningr,isTurningl,countTurning
    if not(isTurningr) or countTurning == 0:
        isTurningl = True
        countTurning +=1
        print("Turning left") 
        vitessex = 0
        vitessez = radians(90)

def turn_right():
    global vitessex, vitessez,isTurningl,isTurningr, countTurning
    if not(isTurningl) or countTurning == 0:
        countTurning +=1
        isTurningr
        print("Turning right")
        vitessex = 0
        vitessez = -radians(90)

def move_forward():
    global vitessex, vitessez
    print("Moving forward")
    vitessex = 0.2
    vitessez = 0

def myhook():
    print("\nshutdown time!")
    cmd= Twist()
    cmd.linear.x= 0
    cmd.angular.z = 0
    commandPublisher.publish(cmd)

# Publish velocity commandes:
def interpret_scan(data):
    global vitessex, vitessez,isTurningr, isTurningl, countTurning
    rospy.loginfo('I get scans')
    obstacles= []
    #obstaclesleft= []
    angle= data.angle_min
    for aDistance in data.ranges :
        if 0.1 < aDistance and aDistance < 5.0 :
            aPoint= [ 
                cos(angle) * aDistance, #x
                sin( angle ) * aDistance #y
            ]
            #detect obstacls only front of the robot
            if(angle < radians(FRONT_ANGLE) and angle > radians(-FRONT_ANGLE-2)):
                obstacles.append( aPoint )
            '''
            elif(angle<radians(-FRONT_ANGLE)):
                obstaclesleft.append(aPoint)
            '''
        elif aDistance >= 5.0: #Consider detection > 5m as an obstacle at 5m (it will not be )
            aPoint= [ 
                cos(angle) * 5.0, #x
                sin( angle ) * 5.0 #y
            ]
            #Detect obstacles only front of the robot
            if(angle < radians(FRONT_ANGLE) and angle > radians(-FRONT_ANGLE)):
                obstacles.append( aPoint )
            '''
            elif(angle<radians(-FRONT_ANGLE)):
                obstaclesleft.append(aPoint)
            '''
        angle+= data.angle_increment
    # rospy.loginfo( str(
    #     [ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[0:10] ] 
    # ) + " ... " + str(
    #     [ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[-10:-1] ] 
    # ) )
    #if obstacle forward
    if(any([(value[0]<DIST_LASER_ROBOT) for value in obstacles])) :
        #check if nothing on the left, and if something, go right 
        if(obstacles[0][0]/cos(FRONT_ANGLE)>obstacles[-1][0]/cos(FRONT_ANGLE)):
            turn_right()
        else:
            turn_left()
    else: #if no obstacle, move forward
        isTurningr = False
        isTurningl = False
        countTurning = 0
        move_forward()
    
    move_command()
    

# connect to the topic:
rospy.Subscriber('scan', LaserScan, interpret_scan)

# call the move_command at a regular frequency:
#rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )
# spin() enter the program in a infinite loop
print("Start move.py")
rospy.spin()

rospy.on_shutdown(myhook)