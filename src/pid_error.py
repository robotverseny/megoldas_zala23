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
msg_pid_state = PidState()

def getRange(data, angle):
    # data: single message from topic /scan
    # angle: between -180 to 180 (-90 to +270) degrees, where 0 degrees is directly to the right (see explain.ipynb)
    # Outputs length in meters to object with angle in lidar scan field of view
    if angle > 269.9: # max 270 deg => 270+90
        angle = 269.9
    index = len(data.ranges) * (angle + 90) / ANGLE_RANGE
    dist = data.ranges[int(index)]
    if math.isinf(dist):
        return 0.02
    if math.isnan(dist):
        return 10.0
    return data.ranges[int(index)]

def followSimple(data):
    # data: single message from topic /scan
    # desired_trajetory: desired distance to the right wall [meters]
    global alpha, pubst1, msg_pid_state
    msg_pid_state.header.frame_id = "simple"
    messageS1 = std_msgs.msg.String()
    messageS1.data = "Egyszeru"
    left_d = getRange(data, -30) 
    right_d = getRange(data, 210)
    forward_d = getRange(data, 270)
    messageS1.data += "\nleft_dist: %.1f\n right_dist: %.1f" % (left_d, right_d)
    messageS1.data += "\nforward_d: %.1f" % (forward_d)
    velocity = forward_d * 0.6
    if velocity < 0.3:
        velocity = 0.0
    error = (left_d - right_d) * 0.3
    #error = -0.01
    messageS1.data += "\nsteer: %.1f" % (error)
    curr_dist = 0
    pubst1.publish(messageS1)
    return error, curr_dist, velocity



def followCenter(data):
    # data: single message from topic /scan
    global alpha, pubst1
    messageS1 = std_msgs.msg.String()
    messageS1.data = "Kozepvonal kovetes"

    a = getRange(data,0) ## original +120 +180 deg, now flipped to 0 -60 deg
    b = getRange(data,-60)
    swing = math.radians(60)
    #print "center distances: ", a, b
    alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    #print "Alpha left",math.degrees(alpha)
    messageS1.data += "\nAlpha left %.1f" % (math.degrees(alpha))
    curr_dist1 = b*math.cos(alpha)
    future_dist1 = curr_dist1-CAR_LENGTH*math.sin(alpha)

    a = getRange(data,180) ## original +60 0 deg, now flipped to +180 +210 deg
    b = getRange(data,210)
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

    # Does a simple follow
    error_simple, curr_dist, velocity = followSimple(data)
    error = error_simple
    # This is code bock for center wall follow
    #error_center, curr_dist_center = followCenter(data)
    #error = error_center

    ## msg = PIDInput()
    ## msg.pid_error = error
    ## msg.pid_vel = VELOCITY
    ## pubm.publish(msg)

    
    msg_pid_state.error = error
    msg_pid_state.error_dot = velocity #VELOCITY
    pubm.publish(msg_pid_state)



if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder',anonymous = True)
    rospy.Subscriber("scan",LaserScan,callbackLaser)
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        pubst2.publish(KOZEPISKOLA_NEVE + "(" + KOZEPISKOLA_AZON + ")")
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass