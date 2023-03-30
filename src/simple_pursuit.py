#!/usr/bin/env python
'''
based on 2018 Varundev Suresh Babu (University of Virginia)
                MIT License
'''

from __future__ import print_function
import rospy, tf2_ros, tf2_geometry_msgs
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PointStamped, Transform
from visualization_msgs.msg import Marker
import std_msgs

KOZEPISKOLA_NEVE = "Ismeretlen kozepiskola"
KOZEPISKOLA_AZON = "A00"
ANGLE_RANGE = 360 # LSN10 LIDAR has 360 degrees scan
DESIRED_DISTANCE_RIGHT = 1.0 #0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8 # 0.55
VELOCITY = 1.00 # meters per second
CAR_LENGTH = 0.445 # 0.445 meters
WHEELBASE = 0.3187; # documention based | measured: ~32 cm

## pubm = rospy.Publisher('error', PIDInput, queue_size=10)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pubst1 = rospy.Publisher('pid_data', std_msgs.msg.String, queue_size=10)
pubst2 = rospy.Publisher('kozepiskola', std_msgs.msg.String, queue_size=10)
marker_pub = rospy.Publisher("/debug_marker",Marker,queue_size=1)
prev_steering_err = 0.0
prev_velocity = 0.0
trans = Transform()


marker_points = Marker()
marker_points.header.frame_id = "base_link" # "laser"
marker_points.type = Marker.SPHERE_LIST
marker_points.action = marker_points.MODIFY
marker_points.color.r = 0.0
marker_points.color.g = 0.0
marker_points.color.a = 1.0
marker_points.color.b = 1.0
marker_points.scale.x = 0.1
marker_points.scale.y = 0.1
marker_points.scale.z = 0.1
marker_points.pose.orientation.x = 0.0
marker_points.pose.orientation.y = 0.0
marker_points.pose.orientation.z = 0.0
marker_points.pose.orientation.w = 1.0


# pure pursuit steering angle calc
def calcPursuitAngle(goal_x, goal_y):
    alpha = math.atan2(goal_y, goal_x)
    lookahead_distance = math.sqrt(pow(goal_x, 2) + pow(goal_y, 2))
    steering_angle = math.atan2(2.0 * WHEELBASE * np.sin(alpha) / (lookahead_distance), 1)
    return steering_angle

def calcPointPos(range, angle):
    x1 = range * math.cos(angle)
    y1 = range * math.sin(angle)
    return x1, y1

def getDistance(ranges, angles):
    global marker_points
    if(len(ranges) > 50):
        center1_min_index = np.where(math.radians(160) < angles)[0][0]
        center1_max_index = np.where(math.radians(179.9) < angles)[0][0]
        tmp1 = np.arange(center1_min_index, center1_max_index, 1)
        center2_min_index = np.where(math.radians(-179.9) < angles)[0][0]
        center2_max_index = np.where(math.radians(-160) < angles)[0][0]
        tmp2 = np.arange(center2_min_index, center2_max_index, 1)
        tmp = np.concatenate((tmp1, tmp2))
        max_x = -10.0
        for t in tmp:
            point = Point()
            point.x, point.y = calcPointPos(ranges[t], angles[t])
            if not math.isinf(point.x):
                # find max (flipped upside min)
                if point.x > max_x:
                    max_x = point.x
            """
            # debug
            if not math.isinf(point.x):
                point.z = 1
                marker_points.points.append(point)
            """
        if math.isinf(max_x):
            max_x = -5.0
        # within 40 cm reverse - tolatas    
        if max_x > -0.4:
            max_x = 0.5
            #print("tolatas - backward motion")            
        """
        # debug
        marker_points.header.frame_id = "laser"
        """
        distance = max_x
    else: 
        distance = 0.4
    return distance

def getAngle(ranges, angles):
    global marker_points
    if(len(ranges) > 50):
        left1_min_index = np.where(math.radians(-150) < angles)[0][0]
        left1_max_index = np.where(math.radians(-120) < angles)[0][0]
        tmp = np.arange(left1_min_index, left1_max_index, 1)
        left_d = -10.0
        for t in tmp:
            point = Point()
            point.x, point.y = calcPointPos(ranges[t], angles[t])
            if not math.isinf(point.y):
                # find max
                if left_d < point.y:
                    left_d = point.y
            """
            # debug
            if not math.isinf(point.x):
                point.z = 1
                marker_points.points.append(point)
            """
        """
        # debug
        marker_points.header.frame_id = "laser"
        """


        right1_min_index = np.where(math.radians(120) < angles)[0][0]
        right1_max_index = np.where(math.radians(150) < angles)[0][0]
        tmp = np.arange(right1_min_index, right1_max_index, 1)
        right_d = 10.0
        for t in tmp:
            point = Point()
            point.x, point.y = calcPointPos(ranges[t], angles[t])
            # find min
            if not math.isinf(point.y):
                if point.y < right_d:
                    right_d = point.y
            """
            # debug
            if not math.isinf(point.x):
                point.z = 1
                marker_points.points.append(point)
            """
        """
        # debug
        marker_points.header.frame_id = "laser"
        """
        angle = (left_d + right_d) / 2
        if math.isinf(right_d):
            right_d = -99.0
            angle = 0.0
        if math.isinf(left_d):
            left_d = 99.0
            angle = 0.0
    else: 
        angle = 0.0

    return angle, left_d, right_d



def followSimple(data):
    # data: single message from topic /scan
    # desired_trajetory: desired distance to the right wall [meters]
    global pubst1, marker_points, prev_steering_err, prev_velocity
    messageS1 = std_msgs.msg.String()
    messageS1.data = "Egyszeru_pursuit"
    angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
    if (len(angles) - len(data.ranges) != 0):
        rospy.logwarn("angles and ranges lenght differ")

    target_distance = getDistance(data.ranges, angles)
    target_angle, left_d, right_d = getAngle(data.ranges, angles)

    point = Point()
    point.x = target_distance
    point.y = target_angle
    point_st = PointStamped()
    point_st.point = point

    try:
        point_base_link_frame = tf2_geometry_msgs.do_transform_point(point_st, trans)
        point_base_link_frame.point.x *= 0.9 # reduce
        # debug
        marker_points.points.append(point_base_link_frame.point)
    except:
        None


    marker_pub.publish(marker_points)

    messageS1.data += "\ntarget_angle: %.1f" % (target_angle)
    messageS1.data += "\nr: %.1f l: %.1f" % (right_d, left_d) 
    messageS1.data += "\nforward_d: %.1f" % (target_distance)
    velocity = -1.0 * target_distance
    try:
        steering_err = calcPursuitAngle(point_base_link_frame.point.x, point_base_link_frame.point.y)
    except:
        steering_err = calcPursuitAngle(1, -1)
        rospy.loginfo("err")
    messageS1.data += "\nsteer: %.1f" % (steering_err)
    pubst1.publish(messageS1)
    marker_points.points = []
    steering_err = (steering_err + prev_steering_err) / 2
    velocity = (velocity + prev_velocity) / 2
    prev_steering_err = steering_err
    prev_velocity = velocity
    return steering_err, velocity


def callbackLaser(data):

    # Does a simple follow
    error_steering, velocity = followSimple(data)


    msg_cmd = Twist()
    msg_cmd.linear.x = velocity * 0.5 # TODO: test low speed
    msg_cmd.angular.z = error_steering # angle
    pub.publish(msg_cmd)    




if __name__ == '__main__':
    first_run = True
    rospy.init_node('dist_finder',anonymous = True)
    rospy.Subscriber("scan",LaserScan,callbackLaser)
    rospy.loginfo("Simple pursuit node started")
    """
    rosrun tf tf_echo base_link laser
    - Translation: [0.260, 0.000, 0.228]
    - Rotation: in Quaternion [0.000, 0.000, 1.000, 0.001]
                in RPY (radian) [0.000, -0.000, 3.140]
                in RPY (degree) [0.000, -0.000, 179.909]
    header:
        frame_id: "base_link"
    child_frame_id: "laser"
    transform:
        translation:
            x: 0.26
            y: 0.0
            z: 0.228
        rotation:
            x: 0.0
            y: 0.0
            z: 0.999999682932
            w: 0.000796326710733
    """
    rate = rospy.Rate(2) # 2hz
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)

    while not rospy.is_shutdown():
        if first_run:
            try:
                trans = buf.lookup_transform("base_link", "laser", rospy.Time.now())
                rospy.loginfo("Got laser fransform")
                first_run = False
                #rospy.loginfo(trans)
            except:
                trans.translation.x = 0.26
                trans.translation.y = 0.0
                trans.translation.z = 0.228
                trans.rotation.x = 0.0
                trans.rotation.y = 0.0
                trans.rotation.z = 0.999999682932
                trans.rotation.w = 0.000796326710733
                #rospy.loginfo(trans)
                #rospy.logerr("No transform to laser assuming original!")
        pubst2.publish(KOZEPISKOLA_NEVE + "(" + KOZEPISKOLA_AZON + ")")
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass