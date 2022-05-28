#! /usr/bin/python2
import sys
import roslib
import rospy
import cv2
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf
from math import *

global odom, pose_with_covariance
global pos
count=0
pos = 0
odom = Odometry()
pose_with_covariance = PoseWithCovarianceStamped()
br = tf.TransformBroadcaster()

rospy.init_node('odom_publisher')
# pub_odom = rospy.Publisher("/m6/odom", Odometry, queue_size=1)
# pub_pose = rospy.Publisher("/m6/pose_with_cov", PoseWithCovarianceStamped, queue_size=1)

# def header_match_callback(data):
#     global odom
#     odom.header.time = data.header
#     odom.header.frame_id = 'odom'

def odom_callback(data):
    global pos, odom, br, count, pose_with_covariance
    if pos == 0:
        for i in range (len(data.name)):
            if data.name[i] == "robot":
                pos = i
                break

    # odom.header.stamp = rospy.Time.now()
    # odom.header.seq = count
    # odom.header.frame_id = "odom"
    #
    # odom.child_frame_id = "base_link"

    a = data.pose[pos].orientation.x
    b = data.pose[pos].orientation.y
    c = data.pose[pos].orientation.z
    d = data.pose[pos].orientation.w
    l = [a, b, c, d]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)
    # pitch = pitch-0.34
    # a, b, c, d = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    # odom.pose.pose.orientation.x = a
    # odom.pose.pose.orientation.y = b
    # odom.pose.pose.orientation.z = c
    # odom.pose.pose.orientation.w = d

    x = data.pose[pos].position.x
    y = data.pose[pos].position.y
    z = data.pose[pos].position.z

    # x = x+1
    # y = y-2

    # x = x + 0.15*cos(yaw)
    # y = y + 0.15*sin(yaw)
    # z = z + 08

    # odom.pose.pose.position.x = x
    # odom.pose.pose.position.y = y
    # odom.pose.pose.position.z = z
    #
    # odom.pose.covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #
    # odom.twist.covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #
    # # odom.pose.pose.position = data.pose[pos].position
    # odom.twist.twist = data.twist[pos]
    # # pub_odom.publish(odom)
    # count+=1

    br.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(roll, pitch, yaw), rospy.Time.now(), "base_link", "map")
    # print(rospy.Time.now())

    # pose_with_covariance.header.seq = count
    # pose_with_covariance.header.stamp = rospy.Time.now()
    # odom.header.frame_id = "odom"
    #
    # pose_with_covariance.pose.pose.position.x = x
    # pose_with_covariance.pose.pose.position.y = y
    # pose_with_covariance.pose.pose.position.z = z
    #
    # pose_with_covariance.pose.pose.orientation.x = a
    # pose_with_covariance.pose.pose.orientation.y = b
    # pose_with_covariance.pose.pose.orientation.z = c
    # pose_with_covariance.pose.pose.orientation.w = d
    #
    # pose_with_covariance.pose.covariance = [0]*36
    #
    # pub_pose.publish(pose_with_covariance)

def main():
    # rospy.Subscriber("/realsense_d435/camera/color/image_raw", Image, header_match_callback, queue_size=1)
    rospy.Subscriber("/gazebo/model_states", ModelStates, odom_callback, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
