#! /usr/bin/python3

import hebi
import numpy as np
import time
import copy
import setup
import rospy
from Tools.transforms import *
from Tools.PIDController import PIDController
from CPG.updateCPGStance import *
# from CPG.newCPG import *
from CPG.kuramoto_CPG import *
from CPG.limitValue import *
from CPG.calculate_cx_a import *
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
import math
from Tools.stability import *
from geometry_msgs.msg import WrenchStamped
import matplotlib.path as mplPath


global new_direction, keyboard_input, torque, land_y

new_direction = '0'
keyboard_input = 'w'
foot_collision = [0] * 19
jointPositions = np.zeros((1, 18))
flag_leg = [False, False, False, False, False, False]
torque = 0
land_y = np.zeros(6)


def joint_callback(data):
    pos = np.array(data.position)
    pos = pos.reshape(1, 18)
    pos = pos.reshape(3, 6)
    pos = pos.T
    pos = pos.reshape(1, 18)
    effort = np.array(data.effort)
    # print("effort",effort)
    jointPositions = pos


def torque_sensor_callback(data):
    global torque
    torque = np.array(data.wrench.torque.y)
    # print("torque",torque)

def land_y_callback(data):
    global land_y
    land_y = np.array(data.data)

def callback(data):
    global new_direction
    # rospy.loginfo(rospy.get_caller_id()+"Heard %s",data.data)
    new_direction = data.data


def keyboard_callabck(data):
    global keyboard_input
    keyboard_input = data.data
    # print(type(data.data))


def leg_correct_callback(data):
    global foot_collision
    foot_collision = np.array(data.data)


# get pose feedback from Gazebo

global q1_x, q1_y, q1_z, q1_w
q1_x = 0
q1_y = 0
q1_z = 0
q1_w = 1

from gazebo_msgs.msg import ModelStates


def gazebo_pose_callback(data):
    global q1_x, q1_y, q1_z, q1_w
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

global pred_foothold
pred_foothold = np.zeros([19])
def foothold_callback(data):
    global pred_foothold
    pred_foothold = np.array(data.data)


# Setup Modules and Kinematics Object
from setup.xMonsterKinematics import *

xmk = HexapodKinematics()

# initialize the complementary filter object
rospack = rospkg.RosPack()
pkgPath = rospack.get_path('xMonsterCPG')
offsets = np.load(pkgPath + '/src/xMonsterCPG/setup/setupFiles/offsets.npy', allow_pickle=True, encoding='latin1')
rospy.init_node('laptop_cpg')

T_elaspsed = 0
T = 5000  # Time in seconds code operates for
nIter = int(round(T / 0.01))

#      \ ____ /           x
#       |    |            |
#     --|    |--     y ___|
#       |____|
#      /      \

# creating cpg dict
cpg = {
    'initLength': 0,
    's': 0.14 * np.ones(6),    # m
    # 'h': 0.23,
    # 'nomX': np.array([0.40, 0.40, 0.0, 0.0, -0.40, -0.40]),
    'nomX': np.array([0.51589, 0.51589, 0.0575, 0.0575, - 0.45839, - 0.45839]),
    'nomY': np.array([0.23145, - 0.23145, 0.5125, - 0.5125, 0.33105, - 0.33105]),
    'h': 0.3249,
    # 'nomY': np.array([0.43158, -0.43158, 0.51276, -0.51276,  0.33118, -0.33118]),
    's1OffsetY': np.array(
        [0.2375 * sin(pi / 6), -0.2375 * sin(pi / 6), 0.1875, -0.1875, 0.2375 * sin(pi / 6), -0.2375 * sin(pi / 6)]),
    # robot measurement;  distance on y axis from robot center to base_actuator
    's1OffsetAngY': np.array([-pi / 3, pi / 3, 0, 0, pi / 3, -pi / 3]),
    'n': 2,  # limit cycle shape 2:standard, 4:super
    'b': np.array([0.8, 0.8, 0.8, 0.8, 0.8, 0.8]),
    # np.array([.4, .4, .4, .4, .4, .4]), #TUNEABLE: step height in radians %1.0
    'scaling': 10,  # TUNEABLE: shifts the units into a reasonable range for cpg processing (to avoid numerical issues)
    'shouldersCorr': np.array([-1, 1, -1, 1, -1, 1]),
    'phase_lags': np.array([pi, pi, 0, pi, 0, pi]),
    'dynOffset': np.zeros([3, 6]),  # Offset on joints developed through constraining
    'dynOffsetInc': np.zeros([3, 6]),  # Increment since last iteration
    'x': np.zeros([nIter, 6]),  # TUNEABLE: Initial CPG x-positions
    'y': np.zeros([nIter, 6]),  # TUNEABLE: Initial CPG y-positions
    'x0': np.zeros([1, 6]),  # limit cycle center x
    'y0': np.zeros([6, 6]),  # limit cycle center y
    'legs': np.zeros([1, 18]),  # Joint angle values
    'elbowsLast': np.zeros([1, 6]),  # elbow values
    'torques': np.zeros([1, 18]),  # Joint torque values
    'torqueOffsets': offsets[2],  # Joint torque offset values
    'gravCompTorques': np.zeros([1, 18]),  # Joint torque values
    'forces': np.zeros([3, 6]),  # ee force values
    'gravCompForces': np.zeros([3, 6]),  # Joint torque values
    'forceStance': np.zeros([1, 6]),  # grounded legs determined by force
    'CPGStance': np.array([False, False, False, False, False, False]),
    # grounded legs determined by position (lower tripod)
    'CPGStanceDelta': np.zeros([1, 6]),  # grounded legs determined by position (lower tripod)
    'CPGStanceBiased': np.zeros([1, 6]),  # grounded legs determined by position (lower tripod)
    'comm_alpha': 1.0,  # commanded alpha in the complementary filter (1-this) is the measured joint angles
    'move': False,
    # true: walks according to cpg.direction, false: stands in place (will continue to stabilize); leave to true for CPG convergence
    'xmk': xmk,  # Snake Monster Kinematics object
    'pose': np.eye(3),  # %SO(3) describing ground frame w.r.t world frame
    'R': SE3(np.eye(3), [0, 0, 0]),  # SE(3) describing body correction in the ground frame
    'G': np.eye(3), #roty(-0.15)[0:3, 0:3],   #   # # describing ground frame w.r.t world frame
    'tp': np.zeros([4, 1]),
    'dynY': 0,
    'vY': 0,
    'direction': 'forward',
    'fullStepLength': 20000,
    't': 0,
    'dx': 0,  # cpg.dx
    'dy': 0   # cpg.dy
}


def command_callback(data):
    cpg['direction'] = data.data
    # cpg['t'] = cpg['initLength'] + 1
    # print(cpg['direction'])

# _(self,vec,kP,kI,alpha,kD,limP,limI,limD,limTot)
# cpg['pid'] = PIDController([3,6],0.0,1.5,0.005,0.01, pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling']); # real robot
# cpg['pid'] = PIDController([3, 6], 0.6, 1, 0.0005, 0.005, pi / 2 * cpg['scaling'], pi / 2 * cpg['scaling'],pi / 2 * cpg['scaling'], pi / 2 * cpg['scaling'])
cpg['pid'] = PIDController([3, 6], 0.0, 1, 0.015, 0.00, pi / 2 * cpg['scaling'], pi / 2 * cpg['scaling'],pi / 2 * cpg['scaling'], pi / 2 * cpg['scaling'])
cpg['T'] = SE3(np.eye(3), np.array([0, 0, cpg['h']]))  # SE(3) describing desired body orientation in the ground frame

# FIXED GROUND ASSUMPTION
GG = rotx(0 / 180 * pi)  # R: This thing is supposed to store the orientation of the ground
[Tx, _, _] = eulerSO3(GG)
actual_bh = 0.25
Xoffset = 0.08  # based on fixed ground  R: This is essentialy to push the robot forward in y, based on the inclination
desired_transY = -(actual_bh * np.tan(Tx) + Xoffset)  # R: Compute the desired "forward push" in y
cpg['eePos'] = np.vstack(
    (cpg['nomX'], cpg['nomY'], -cpg['h'] * np.ones([1, 6])))  # R: Compute the EE positions in body frame

ang = cpg['xmk'].getLegIK(cpg['eePos'])  # R: This gives the angles corresponding to each of the joints

cpg['nomOffset'] = np.reshape(ang[0:18], [6, 3]).T  # R: These are the angles corresponding to the rest position


pi = math.pi

stride_length = 0.14
cpg['foothold_offsetY'] = np.array([0.40145, - 0.40145, 0.5125, - 0.5125, 0.32105, - 0.32105])
foothold_offsetY = cpg['foothold_offsetY']

dist = cpg['foothold_offsetY'] - cpg['s1OffsetY']  # the distance between foot ground trajectory to base actuator


leg_offset = np.array([pi / 3, pi / 3, 0, 0, -pi / 3, -pi / 3]) + 0.173 * np.ones([6])
# leg_offset = np.array([pi/3, pi/3, 0, 0, pi/3, pi/3]) + 0.173 * np.array([1,1,1,1,-1,-1])   # to avoid the back leg tan(theta) numerical issue
cx = np.array([-1 / 4, -1 / 4, -1/9, -1/9, 0, 0]) * math.pi
a = calculate_a(cpg, cx)
a = np.reshape(a, 6)

# ground y axis angle
yaw = 0   # yaw > 0 head down , yaw < 0 head up

cy = np.array([0, 0, 0, 0, 0, 0]) * math.pi

p = np.tan(leg_offset + cx)

leg_left_ground_t = np.zeros([6])
pre_topic_t = 0
x_correction = np.array([0., 0., 0., 0., 0., 0.])  # move land point forward in meters
x_correction_sum = np.zeros(6)

y_correction = np.array([0., 0., 0., 0., 0., 0.])  # move land point forward in meters
y_correction_sum = np.zeros(6)

flag_change = False

cpg['a'] = a * cpg['scaling']
cpg['cx'] = cx * cpg['scaling']
cpg['cy'] = cy * cpg['scaling']

cpg['b'] = cpg['b'] * cpg['scaling']
cpg['nomOffset'] = cpg['nomOffset'] * cpg['scaling']

# CPG Initialization
cpg['wStance'] = 0.80 * 6  # cpg anglular speed [rad]/[sec]
cpg['wSwing'] = cpg['wStance'] * 6.0
cpg['K'] = np.array([[0, -1, -1, 1, 1, -1],
                     [-1, 0, 1, -1, -1, 1],
                     [-1, 1, 0, -1, -1, 1],
                     [1, -1, -1, 0, 1, -1],
                     [1, -1, -1, 1, 0, -1],
                     [-1, 1, 1, -1, -1, 0]])

cpg['phi'] = [[0, np.pi, np.pi, 0, 0, np.pi],
              [np.pi, 0, 0, np.pi, np.pi, 0],
              [np.pi, 0, 0, np.pi, np.pi, 0],
              [0, np.pi, np.pi, 0, 0, np.pi],
              [0, np.pi, np.pi, 0, 0, np.pi],
              [np.pi, 0, 0, np.pi, np.pi, 0]]

u = np.array([-pi / 5, 7 * pi / 5, 6 * pi / 5, 9 * pi / 5, 8 * pi / 5, 7 * pi / 5])

cpg['x'][0, :] = (a * np.array([1, -1, -1, 1, 1, -1]) + cx) * cpg['scaling']  # R: Initialize the x and y values of the cpg cycle
cpg['y'][0, :] = np.zeros(6)  # np.array([-0.7216 ,   0.7216 ,   0.7216,   -0.7216,   -0.7216,    0.7216]);
cpg['xGr'] = cpg['x'][0, :]  # R: x or first joint angle values, corresponding to grounded legs

cpg['dxoffset'] = np.zeros(6)
cpg['phase'] = np.zeros(6)

# done initializing cpg
ang_rest = np.zeros(18)
ang_rest[0:18:3] = (a * np.array([1, -1, -1, 1, 1, -1]) + cx) * cpg['shouldersCorr']
ang_rest[1:18:3] = cpg['nomOffset'][1, :]/cpg['scaling']
ang_rest[2:18:3] = cpg['nomOffset'][2, :]/cpg['scaling']

prevFeedbackTime = time.time()
cpg['time_previous'] = time.time()
rate = rospy.Rate(50.0)


rospy.Subscriber("m6/joint_states", JointState, joint_callback)
# rospy.Subscriber("/m6/sensors/elbow1/torque", WrenchStamped, torque_sensor_callback)
rospy.Subscriber("/hexapod/direction/command", String, command_callback)

rospy.Subscriber("further_direction", String, callback, queue_size=1)
rospy.Subscriber("leg_need_change", Float32MultiArray, leg_correct_callback, queue_size=1)
rospy.Subscriber("predict_land_y", Float32MultiArray, land_y_callback, queue_size=1)
rospy.Subscriber("/keys", String, keyboard_callabck, queue_size=1)
rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_pose_callback)

rospy.Subscriber("/foothold_in_map", Float32MultiArray, foothold_callback, queue_size=1)

pub_base1 = rospy.Publisher("/m6/base1_position_controller/command", Float64, queue_size=1)
pub_base2 = rospy.Publisher("/m6/base2_position_controller/command", Float64, queue_size=1)
pub_base3 = rospy.Publisher("/m6/base3_position_controller/command", Float64, queue_size=1)
pub_base4 = rospy.Publisher("/m6/base4_position_controller/command", Float64, queue_size=1)
pub_base5 = rospy.Publisher("/m6/base5_position_controller/command", Float64, queue_size=1)
pub_base6 = rospy.Publisher("/m6/base6_position_controller/command", Float64, queue_size=1)

pub_shoulder1 = rospy.Publisher("/m6/shoulder1_position_controller/command", Float64, queue_size=1)
pub_shoulder2 = rospy.Publisher("/m6/shoulder2_position_controller/command", Float64, queue_size=1)
pub_shoulder3 = rospy.Publisher("/m6/shoulder3_position_controller/command", Float64, queue_size=1)
pub_shoulder4 = rospy.Publisher("/m6/shoulder4_position_controller/command", Float64, queue_size=1)
pub_shoulder5 = rospy.Publisher("/m6/shoulder5_position_controller/command", Float64, queue_size=1)
pub_shoulder6 = rospy.Publisher("/m6/shoulder6_position_controller/command", Float64, queue_size=1)

pub_elbow1 = rospy.Publisher("/m6/elbow1_position_controller/command", Float64, queue_size=1)
pub_elbow2 = rospy.Publisher("/m6/elbow2_position_controller/command", Float64, queue_size=1)
pub_elbow3 = rospy.Publisher("/m6/elbow3_position_controller/command", Float64, queue_size=1)
pub_elbow4 = rospy.Publisher("/m6/elbow4_position_controller/command", Float64, queue_size=1)
pub_elbow5 = rospy.Publisher("/m6/elbow5_position_controller/command", Float64, queue_size=1)
pub_elbow6 = rospy.Publisher("/m6/elbow6_position_controller/command", Float64, queue_size=1)

pub_control_state = rospy.Publisher("/m6/control_state", Float64, queue_size=1)

control_state = 0.0
while not rospy.is_shutdown():
    # Time feedback
    timeNow = time.time()
    delte_time = timeNow - prevFeedbackTime
    dt = max(min(delte_time, 0.04), 0.01)  # Ensure 25Hz-100Hz for CPG stability
    prevFeedbackTime = timeNow

    # Get robot pose
    matrix = R.from_quat([q1_x, q1_y, q1_z, q1_w], normalized=False)
    quta_matrix = matrix.as_matrix()
    cpg['pose'] = quta_matrix
    cpg['land_y'] = land_y

    # stride length = 0.15m
    a_ini = a * cpg['scaling']
    cx_ini = cx * cpg['scaling']

    # Position feedback
    cpg['legs'] = cpg['comm_alpha'] * cpg['legs'] + (1 - cpg[
        'comm_alpha']) * jointPositions  # R: Basically weighing the cpg computation and the position feedback

    cpg = updateCPGStance(cpg, cpg['t'])

    # renew the cx and a when the leg left the ground
    for i in range(6):
        if cpg['t'] > leg_left_ground_t[i] + 20:
            if cpg['CPGStanceDelta'][i] == True:
                if cpg['CPGStance'][i] == False:
                    print("leg ",i,' left ground')
                    cpg['a'][i] = a_ini[i]
                    cpg['cx'][i] = cx_ini[i]
                    cpg['foothold_offsetY'] = foothold_offsetY
                    x_correction_sum[i] = 0
                    y_correction_sum[i] = 0
                    # due to time dely the x demention cannot correct at the time leg left the ground
                    flag_leg[i] = False
                    leg_left_ground_t[i] = cpg['t']     # make sure that renew the cx and a at next loop

    # foot_collection = [0]* 12 first 6 elements are the legs need to be changed, last 6 elements are the gradients of the cost map
    # stairs width = 0.25m   height = 0.1m
    foot_collision_copy = np.copy(foot_collision)
    foot_offset_y = np.array(foot_collision_copy[12:18])

    # limit the adjust range in y dimension
    topic_t = foot_collision_copy[18]

    # set the x_correction
    if topic_t > pre_topic_t:
        for leg in range(6):
            if cpg['dy'][leg] > 0:     # only adjust a and cx when leg is rising
                if foot_collision_copy[leg]:
                    flag_leg[int(leg)] = True
                    flag_change = True
                    pre_topic_t = cpg['t']
                    y_correction_sum[leg] = - 0.8 * foot_offset_y[leg]
                    if foot_collision_copy[leg + 6] > 0:
                        x_correction[leg] = -0.5 * foot_collision[leg + 6]  #- 0.02
                        x_correction_sum[leg] = x_correction_sum[leg] - 0.2 * foot_collision[leg + 6]  #- 0.02
                    else:
                        x_correction[leg] = -0.5 * foot_collision[leg + 6]  #+ 0.02
                        x_correction_sum[leg] = x_correction_sum[leg] - 0.2 * foot_collision[leg + 6] # + 0.02

    if flag_change:
        x_correction_sum = np.maximum(np.minimum(x_correction_sum, 0.10), -0.10)
        y_correction_sum = np.maximum(np.minimum(y_correction_sum, 0.07), -0.07)
        cpg['foothold_offsetY'] = foothold_offsetY + y_correction_sum
        cx_cal = calculate_cx(cpg, x_correction_sum, cx, a)
        a_cal = calculate_a(cpg, cx_cal)
        a_cal = np.reshape(a_cal, 6)
        # print("update!!!")
        cpg['a'][flag_leg] = a_cal[flag_leg] * cpg['scaling']
        cpg['cx'][flag_leg] = cx_cal[flag_leg] * cpg['scaling']
        flag_change = False


    # body translation
    if cpg['t'] > 100:
        if cpg['phase'][0] < pi / 2 and cpg['phase'][0] > (pi / 2 - 0.02):
            foothold_base = pred_foothold[0:18].reshape(6, 3).T
            foothold_binary = np.array([0 if foothold_base[0,i]==0 else 1 for i in range(len(foothold_base[0]))])
            print('-------------',np.count_nonzero(foothold_binary == 1))
            if np.count_nonzero(foothold_binary == 1) > 2:
                support_polygon = convex_hull(foothold_base, foothold_binary)
                if mplPath.Path(support_polygon).contains_point(np.array([cpg['s'][0]/2, 0])):
                    print('--------------------stable-------------------')
                else:
                    print('------------------body translating--------------------')
                    dis = cross_point(support_polygon, np.array([cpg['s'][0]/2, 0]))
                    cpg['cx'] = calculate_cx(cpg, -np.ones(6)*dis, cpg['cx']/cpg['scaling'], cpg['a']/cpg['scaling'])*cpg['scaling']
                    cpg['a'] = calculate_a(cpg, cpg['cx']/cpg['scaling']).reshape(6)*cpg['scaling']

        if cpg['phase'][1] < pi / 2 and cpg['phase'][1] > (pi / 2 - 0.02):
            foothold_base = pred_foothold[0:18].reshape(6, 3).T
            foothold_binary = np.array([0 if foothold_base[0,i]==0 else 1 for i in range(len(foothold_base[0]))])
            print('-------------',np.count_nonzero(foothold_binary == 1))
            if np.count_nonzero(foothold_binary == 1) == 3:
                support_polygon = convex_hull(foothold_base, foothold_binary)
                if mplPath.Path(support_polygon).contains_point(np.array([cpg['s'][0]/2, 0])):
                    print('--------------------stable-------------------')
                    print(support_polygon)
                else:
                    print('------------------body translating--------------------')
                    dis = cross_point(support_polygon, np.array([cpg['s'][0] / 2, 0]))
                    cpg['cx'] = calculate_cx(cpg, -np.ones(6) * dis, cpg['cx'] / cpg['scaling'],
                                             cpg['a'] / cpg['scaling']) * cpg['scaling']
                    cpg['a'] = calculate_a(cpg, cpg['cx'] / cpg['scaling']).reshape(6) * cpg['scaling']


    x_correction = np.zeros(6)

    # CPG Controller
    cpg, positions = CPG(cpg, cpg['t'], dt)
    t = cpg['t']
    # print("topic time : ", foot_collision_copy[18])
    print("cpg time : ", cpg['t'])
    print("---------------------------------")


    if cpg['t'] > cpg['initLength']+50:
        control_state = 1.0

    if cpg['t'] > (cpg['initLength'] + 50):
        cpg['move'] = True
        commanded_position = cpg['legs'][0, :]

        # set the desired pose
        # cpg['G'] = roty(yaw)[0:3, 0:3]

    else:
        commanded_position = ang_rest[0:18]

    if keyboard_input == 'w':
        control_state = 0.0
        cpg.update({'direction': 'forward'})
    elif keyboard_input == 'a':
        control_state = 0.0
        cpg.update({'direction': 'left'})
    elif keyboard_input == 'd':
        control_state = 0.0
        cpg.update({'direction': 'right'})
    else:
        control_state = 1.0
        cpg.update({'move': 'False'})

    pub_control_state.publish(control_state)

    pub_base1.publish(commanded_position[0])
    pub_base2.publish(commanded_position[3])
    pub_base3.publish(commanded_position[6])
    pub_base4.publish(commanded_position[9])
    pub_base5.publish(commanded_position[12])
    pub_base6.publish(commanded_position[15])

    pub_shoulder1.publish(commanded_position[1])
    pub_shoulder2.publish(commanded_position[4])
    pub_shoulder3.publish(commanded_position[7])
    pub_shoulder4.publish(commanded_position[10])
    pub_shoulder5.publish(commanded_position[13])
    pub_shoulder6.publish(commanded_position[16])

    pub_elbow1.publish(commanded_position[2])
    pub_elbow2.publish(commanded_position[5])
    pub_elbow3.publish(commanded_position[8])
    pub_elbow4.publish(commanded_position[11])
    pub_elbow5.publish(commanded_position[14])
    pub_elbow6.publish(commanded_position[17])

    rate.sleep()

    cpg['t'] += 1


