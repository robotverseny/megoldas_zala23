#!/usr/bin/env python
'''
2018 Varundev Suresh Babu (University of Virginia)
                MIT License
'''

from __future__ import print_function
import rospy
import math
#import numpy as np
from sensor_msgs.msg import LaserScan
## from simulator.msg import PIDInput
from control_msgs.msg import PidState
import std_msgs

KOZEPISKOLA_NEVE = "Ismeretlen kozepiskola"
KOZEPISKOLA_AZON = "A00"
ANGLE_RANGE = 360 # LSN10 LIDAR has 360 degrees scan
DESIRED_DISTANCE_RIGHT = 1.0 #0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8 # 0.55
VELOCITY = 1.00 # meters per second
CAR_LENGTH = 0.445 # 0.445 meters


## pubm = rospy.Publisher('error', PIDInput, queue_size=10)
pubm = rospy.Publisher('error', PidState, queue_size=10)
pubst1 = rospy.Publisher('pid_data', std_msgs.msg.String, queue_size=10)
pubst2 = rospy.Publisher('kozepiskola', std_msgs.msg.String, queue_size=10)


def getRange(data, angle):
    # data: single message from topic /scan
    # angle: between -180 to 180 degrees, where 0 degrees is TODO: (directly to the right??s)
    # Outputs length in meters to object with angle in lidar scan field of view
    if angle > 179.9:
        angle = 179.9
    index = len(data.ranges) * (angle + 90) / ANGLE_RANGE
    dist = data.ranges[int(index)]
    if math.isinf(dist):
        return 10.0
    if math.isnan(dist):
        return 4.0
    return data.ranges[int(index)]

def followRight(data, desired_trajectory):
    # data: single message from topic /scan
    # desired_trajetory: desired distance to the right wall [meters]
    global alpha, pubst1
    messageS1 = std_msgs.msg.String()
    messageS1.data = "Jobb oldal kovetes"
    a = getRange(data,60)
    b = getRange(data,0)
    swing = math.radians(60)
    alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    messageS1.data += "\na: %.1f b: %.1f" % (a, b)
    messageS1.data += "\nAlpha left %.1f" % (math.degrees(alpha))
    curr_dist = b*math.cos(alpha)

    future_dist = curr_dist + CAR_LENGTH * math.sin(alpha)
    messageS1.data += "\nRight: %.2f" % (future_dist)
    error = desired_trajectory - future_dist

    messageS1.data += "\nCurrent Distance Left: %.2f" % (curr_dist)
    pubst1.publish(messageS1)
    return error, curr_dist

def followLeft(data, desired_trajectory):
    # data: single message from topic /scan
    # desired_trajectory: desired distance to the left wall [meters]
    global alpha, pubst1
    messageS1 = std_msgs.msg.String()
    a = getRange(data,120)
    b = getRange(data,179.9)
    swing = math.radians(60)
    #print("a","b", a, b)
    messageS1.data = "Bal oldal kovetes\na: %.1f b: %.1f" % (a, b)
    alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    #rospy.loginfo("Alpha left %.1f" % (math.degrees(alpha)))
    #print("Alpha left",math.degrees(alpha))
    messageS1.data += "\nAlpha left %.1f" % (math.degrees(alpha))
    curr_dist = b*math.cos(alpha)

    future_dist = curr_dist - CAR_LENGTH * math.sin(alpha)
    #print("Left : ",future_dist)
    messageS1.data += "\nLeft: %.2f" % (future_dist)
    error = future_dist - desired_trajectory

    #print("Current Distance Left: ", curr_dist)
    messageS1.data += "\nCurrent Distance Left: %.2f" % (curr_dist)
    pubst1.publish(messageS1)
    return error, curr_dist

def followCenter(data):
    # data: single message from topic /scan
    global alpha, pubst1
    messageS1 = std_msgs.msg.String()
    messageS1.data = "Kozepvonal kovetes"

    a = getRange(data,120)
    b = getRange(data,179.9)
    swing = math.radians(60)
    #print "center distances: ", a, b
    alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    #print "Alpha left",math.degrees(alpha)
    curr_dist1 = b*math.cos(alpha)
    future_dist1 = curr_dist1-CAR_LENGTH*math.sin(alpha)

    a = getRange(data,60)
    b = getRange(data,0)
    swing = math.radians(60)
    alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    #print "Alpha right",math.degrees(alpha)
    messageS1.data += "\nAlpha right %.1f" % (math.degrees(alpha))
    curr_dist2 = b*math.cos(alpha)
    future_dist2 = curr_dist2+CAR_LENGTH*math.sin(alpha)

    #print "dist 1 : ",future_dist1
    #print "dist 2 : ",future_dist2

    error = future_dist1 - future_dist2
    #print "Error : ",error
    pubst1.publish(messageS1)
    return error, curr_dist2 - curr_dist1

def callbackLaser(data):
    global error
    global alpha
    global final_direction
    global prev_direction

    #print " "

    # Does a left wall follow
    #error_left, curr_dist_left = followLeft(data, DESIRED_DISTANCE_LEFT)
    #error = error_left

    # Does a right wall follow
    #error_right, curr_dist_right = followRight(data, DESIRED_DISTANCE_RIGHT)
    #error = error_right

    # This is code bock for center wall follow
    error_center, curr_dist_center = followCenter(data)
    error = error_center

    ## msg = PIDInput()
    ## msg.pid_error = error
    ## msg.pid_vel = VELOCITY
    ## pubm.publish(msg)

    msg_pid_state = PidState()
    msg_pid_state.error = error
    msg_pid_state.error_dot = VELOCITY
    pubm.publish(msg_pid_state)



if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder',anonymous = True)
    rospy.Subscriber("scan",LaserScan,callbackLaser)
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        pubst2.publish(KOZEPISKOLA_NEVE + "(" + KOZEPISKOLA_AZON + ")")
        rate.sleep()