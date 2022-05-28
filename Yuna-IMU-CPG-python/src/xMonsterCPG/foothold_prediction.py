#! /usr/bin/python

# foot hold prediction

import rospy
import hebi
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf2_geometry_msgs import PointStamped
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point
from Tools.constrainSE3 import *
from CPG.limitValue import *
from CPG.groundIK import *
from CPG.supelliptRot import *
import setup
import rospy
import math
from std_msgs.msg import Float32MultiArray
from grid_map_msgs.msg import GridMap
import traceback


# np.set_printoptions(threshold=np.inf)
pi = np.pi
import time

from setup.xMonsterKinematics import *
xmk = HexapodKinematics()

global cpg, leg_air, Flag
global cpg_xy, map_array, map_resolution, map_dimension_x, map_dimension_y, map_center_x, map_center_y, dist
cpg = np.zeros(31)    # cpg['a'], cpg['b'], cpg['cx'], cpg['cy'],cpg['t'],cpg['nomOffset'][1,:]
cpg_xy = np.ones(12)
map_array = np.zeros([10000, 10000])
map_resolution = 0.05
map_dimension_x = 80
map_dimension_y = 80
map_center_x = 0
map_center_y = 0
leg_air = np.zeros(6)


Leglength = 0.325
dist = [0.38144999742507935, -0.38144999742507935, 0.512499988079071, -0.512499988079071, 0.33105000853538513, -0.33105000853538513]
Flag = False

def dist_callback(data):
    global dist
    dist = np.array(data.data)

def leg_ground_callback(data):
    global leg_air, Flag
    Flag = True
    leg_air = np.array(data.data)

def cpg_callback(data):
    global cpg
    cpg = np.array(data.data)

def cpgxy_callback(data):
    global cpg_xy, Flag
    Flag = True
    cpg_xy = np.array(data.data)



def grid_callback(grid_msg):
    global map_array, map_resolution, map_dimension_x, map_dimension_y, map_center_x, map_center_y
    grid = grid_msg.data
    map_resolution = grid_msg.info.resolution
    # print(grid_msg.info)
    a = grid[0].data
    map_dimension_x = grid[0].layout.dim[0].size
    map_dimension_y = grid[0].layout.dim[1].size
    b = np.array(a)
    hm_array = b.reshape(map_dimension_x, map_dimension_y)
    hm_array = np.nan_to_num(hm_array)
    map_array = np.flip(np.rot90(hm_array, 2), 0)
    map_center_x = grid_msg.info.pose.position.x
    map_center_y = grid_msg.info.pose.position.y
    # print('x:',map_center_x,'y:',map_center_y)


rospy.init_node('foothold_predict')

topic_1 = 'footfall_trajectory_predict'
publisher_1 = rospy.Publisher(topic_1, MarkerArray, queue_size=1)
pub_landy = rospy.Publisher('predict_land_y', Float32MultiArray, queue_size=1)
pub_foothold = rospy.Publisher('foothold_in_map', Float32MultiArray, queue_size=1)
markerArray_1 = MarkerArray()

# publish the past foothold
topic_2 = 'past_foothold'
publisher_2 = rospy.Publisher(topic_2, MarkerArray, queue_size=1)
markerArray_2 = MarkerArray()

num = 0
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
# rate = rospy.Rate(30.0)
np.set_printoptions(precision=5)
stride_length = 0.14
cpg_dic = {
    'xmk': xmk,
    'nomY': np.array([0.33145, - 0.33145,   0.5125, - 0.5125,   0.33105, - 0.33105]),
    'legs': np.zeros([1,18]) #Joint angle values
      }

rospy.Subscriber("/CPG", Float32MultiArray, cpg_callback, queue_size=1)
rospy.Subscriber("/CPG_xy", Float32MultiArray, cpgxy_callback, queue_size=1)
rospy.Subscriber("/elevation_map_matrix", GridMap, grid_callback, queue_size=1, buff_size=52428800)
rospy.Subscriber("/Foot_dist", Float32MultiArray, dist_callback, queue_size=1)
rospy.Subscriber("/leg_ground", Float32MultiArray, leg_ground_callback, queue_size=1)

while not rospy.is_shutdown():
    try:
        if Flag:
            np.set_printoptions(precision=5)
            cpg_x = cpg_xy[0:6]
            cpg_y = cpg_xy[6:12]
            cpg_copy = np.copy(cpg)
            a = cpg_copy[0:6]
            b = cpg_copy[6:12]
            cx = cpg_copy[12:18]
            cy = cpg_copy[18:24]
            nomoffset = cpg_copy[25:31]

            del markerArray_1.markers[:]

            theta = np.arccos((cpg_x-cx)/(np.sqrt((cpg_x-cx)**2+(cpg_y-cy)**2)))

            # print(theta/pi*180)
            theta[np.isnan(theta)] = 0
            # print(theta)
            leg_air_i = np.nonzero(leg_air)   # eg.leg_air_i = [1, 2, 5]  or [0, 3, 4]

            legs = np.zeros([1, 18])
            # leg_position_map = [[] for i in range(6)]
            # leg_position_local = [[] for i in range(6)]
            cx_orignal = cx / 10
            predict_foothold_map = np.zeros([19])
            land_y = np.zeros(6)

            for i in leg_air_i[0]:                # only predict the leg in the air
                for m in np.arange(0, theta[i], 0.08):   #predict the remain trajectory
                        n = theta[i]-m
                        if n <= math.pi/2:
                            base = a[i]*math.cos(n)+cx[i]
                            shoulder = b[i]*math.sin(n) + cy[i]
                            base = base*(-1)**(i+1) / 10
                            shoulder = shoulder*(-1)**(i+1) / 10 + nomoffset[i]/10

                            cpg_dic['legs'][0,3*i:3*i+3] = [base,shoulder,(-1) ** (i + 1) * math.pi / 2]
                            angs = groundIK_leg(cpg_dic, dist, i)
                            elbow = angs[2]
                            # elbow = angs[2 + i * 3]
                            # cpg['legs'][0, 2 + index * 3] = angs[2 + index * 3] + (cpg['dynOffset'][2, index] / cpg['scaling'])

                            legs = np.zeros([1, 18])
                            legs[0, 3*i: 3*(i+1)] = np.array([base, shoulder, elbow])
                            positions = xmk.getLegPositions_leg(i, legs)
                            P_base = PointStamped()
                            P_base.header.stamp = rospy.Time()
                            P_base.header.frame_id = 'base_link'
                            leg_pos = positions[:, 0].tolist()
                            P_base.point.x, P_base.point.y, P_base.point.z = leg_pos[0] + stride_length / 2 * (1 - math.cos(m)), leg_pos[1], leg_pos[2]
                            # P.point.x, P.point.y, P.point.z = leg_pos[0] + stride_length * m / pi, leg_pos[1], leg_pos[2]

                            # body trans define
                            # if i == 0 or 1:
                            #     if theta[i] < pi / 3 and theta[i] > (pi / 3 - 0.01):
                            #         body_flag = True
                            #         body_trans_msg = Float32MultiArray(data=np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1]))
                            #         pub_body_trans.publish(body_trans_msg)
                            #     else:
                            #         body_flag = False
                            #         body_trans_msg = Float32MultiArray(data=np.array([0]))
                            #         pub_body_trans.publish(body_trans_msg)

                            P_map = tfBuffer.transform(P_base, 'map', timeout=rospy.Duration(1))

                            marker = Marker()
                            marker.header.frame_id = "/base_link"   # "/map"
                            marker.type = marker.SPHERE
                            marker.action = marker.ADD
                            marker.scale.x = 0.02
                            marker.scale.y = 0.02
                            marker.scale.z = 0.02
                            marker.color.a = 1.0
                            if num % 3 == 0:
                                marker.color.r = 1.0
                                marker.color.g = 0.0
                                marker.color.b = 0.0
                            elif num % 3 == 1:
                                marker.color.r = 0.0
                                marker.color.g = 1.0
                                marker.color.b = 0.0
                            elif num % 3 == 2:
                                marker.color.r = 0.0
                                marker.color.g = 0.0
                                marker.color.b = 1.0

                            # calculate under map frame
                            """
                            marker.pose.orientation.w = 1.0
                            marker.pose.position.x = P_map.point.x
                            marker.pose.position.y = P_map.point.y
                            marker.pose.position.z = P_map.point.z
                            
                            p = P_map.point.x - map_center_x  # foot trajectory in robot frame
                            q = P_map.point.y - map_center_y  # foot trajectory in robot frame
                            """
                            # calculate under base_link frame
                            marker.pose.orientation.w = 1.0
                            marker.pose.position.x = P_base.point.x
                            marker.pose.position.y = P_base.point.y
                            marker.pose.position.z = P_base.point.z

                            p = P_base.point.x
                            q = P_base.point.y


                            foot_localmap_x = (map_dimension_y * map_resolution / 2 - (
                                        q)) // map_resolution
                            foot_localmap_y = (map_dimension_x * map_resolution / 2 + (
                                        p)) // map_resolution
                            markerArray_1.markers.append(marker)
                            if map_array[int(foot_localmap_x-1), int(foot_localmap_y-1)] >= (P_map.point.z - 0.01):

                                # markerArray_2.markers.append(marker)
                                predict_foothold_map[3 * i:3 * (i + 1)] = [P_base.point.x, P_base.point.y, P_base.point.z]
                                land_y[i] = shoulder

                                break

                            # leg_position_map[i].append([P_map.point.x, P_map.point.y, P_map.point.z])
                            # leg_position_local[i].append([P.point.x, P.point.y, P.point.z])
            landy_topublish = Float32MultiArray(data=land_y)
            pub_landy.publish(landy_topublish)
            # print('land y in prediction : ',land_y)
            # get the cost_map and calculate the gradient
            predict_foothold_map[18] = cpg_copy[24]

            data_topublish = Float32MultiArray(data=predict_foothold_map)
            pub_foothold.publish(data_topublish)

            id = 0

            # print(leg_air)
            for m in markerArray_1.markers:
                m.id = id
                id += 1

            # for m2 in markerArray_2.markers:
            #     m2.id = id2
            #     id2 += 1

            publisher_1.publish(markerArray_1)
            # publisher_2.publish(markerArray_2)

            # del markerArray.markers[:]
            del markerArray_1.markers[:]

            num += 1
            # rospy.spin()
        else:
            # print("subscribing message ...")
            continue
    except:
        continue
    #     print('traceback.format_exc():\n%s' % traceback.format_exc())
    #     print('finding tf...')
    #     traceback.print_exc()
