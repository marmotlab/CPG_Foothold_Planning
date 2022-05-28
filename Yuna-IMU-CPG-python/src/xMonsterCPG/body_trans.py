#! /usr/bin/python3

import rospy
import hebi
import numpy as np
import setup
import rospy
import math
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from Tools.stability import *
import matplotlib.path as mplPath

pi = np.pi
import time

from setup.xMonsterKinematics import *

xmk = HexapodKinematics()

global cpg, leg_air, Flag, EE_pos
global cpg_xy, map_array, map_resolution, map_dimension_x, map_dimension_y, map_center_x, map_center_y, dist
cpg = np.zeros(31)  # cpg['a'], cpg['b'], cpg['cx'], cpg['cy'],cpg['t'],cpg['nomOffset'][1,:]

leg_air = np.zeros(6)
EE_pos = positions = np.array([[0.50588, 0.63498, 0.1406, -0.00552, -0.47454, -0.54181],
                              [0.2559, -0.25935, 0.60987, -0.5175, 0.31018, -0.42041],
                              [-0.2249, -0.224, -0.2291, -0.2249, -0.2249, -0.223]])

global pose

pose = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])


Flag = False

def joint_callback(data):
    global EE_pos
    pos = np.array(data.position)
    pos = pos.reshape(1, 18)
    pos = pos.reshape(3, 6)
    pos = pos.T
    pos = pos.reshape(1, 18)
    EE_pos = xmk.getLegPositions(pos)


def cpg_callback(data):
    global cpg
    cpg = np.array(data.data)


def leg_ground_callback(data):
    global leg_air, Flag
    Flag = True
    leg_air = np.array(data.data)    # swing =1

def gazebo_pose_callback(data):
    global pos
    num = 0
    for i in range(len(data.name)):
        if data.name[i] == 'robot':
            num = i
    a = data.pose[num].orientation
    # print(data.name[num])
    q1_x = a.x
    q1_y = a.y
    q1_z = a.z
    q1_w = a.w

    matrix = R.from_quat([q1_x, q1_y, q1_z, q1_w], normalized=False)
    pos = matrix.as_matrix()



rospy.init_node('body_translation')



num = 0

# rate = rospy.Rate(30.0)
np.set_printoptions(precision=5)
stride_length = 0.18
cpg_dic = {
    'xmk': xmk,
    'nomY': np.array([0.33145, - 0.33145, 0.5125, - 0.5125, 0.33105, - 0.33105]),
    'legs': np.zeros([1, 18])  # Joint angle values
}

rospy.Subscriber("/CPG", Float32MultiArray, cpg_callback, queue_size=1)
rospy.Subscriber("/leg_ground", Float32MultiArray, leg_ground_callback, queue_size=1)
rospy.Subscriber("m6/joint_states", JointState, joint_callback)
rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_pose_callback)

while not rospy.is_shutdown():

        if Flag:
            np.set_printoptions(precision=5)
            cpg_copy = np.copy(cpg)
            a = cpg_copy[0:6]
            b = cpg_copy[6:12]
            cx = cpg_copy[12:18]
            cy = cpg_copy[18:24]
            nomoffset = cpg_copy[25:31]

            EE_project = np.dot(pose, EE_pos)

            support_polygon = convex_hull(EE_project, np.invert(leg_air.astype(np.int))+2)
            if mplPath.Path(support_polygon).contains_point(np.array([0, 0])):
                print('stable')
            else:
                print('unstable')

            print('test',support_polygon)

            # rospy.spin()
        else:
            # print("subscribing message ...")
            continue

    #     print('traceback.format_exc():\n%s' % traceback.format_exc())
    #     print('finding tf...')
    #     traceback.print_exc()
