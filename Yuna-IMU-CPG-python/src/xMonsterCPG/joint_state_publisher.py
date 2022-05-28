#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
global t
t = 0

def talker():
    global t
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    joint_state = JointState()
    joint_state.header = Header()
    # joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['base1', 'shoulder1', 'elbow1', 'base3', 'shoulder3', 'elbow3', 'base5', 'shoulder5', 'elbow5', 'base2',
                       'shoulder2', 'elbow2', 'base4', 'shoulder4', 'elbow4', 'base6', 'shoulder6', 'elbow6']


    joint_state.velocity = []
    joint_state.effort = []
    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = [math.sin(t), math.sin(t), math.sin(t), math.sin(t), math.sin(t), math.sin(t),
                                math.sin(t), math.sin(t), math.sin(t), math.sin(t), math.sin(t), math.sin(t),
                                math.sin(t), math.sin(t), math.sin(t), math.sin(t), math.sin(t), math.sin(t)]
        pub.publish(joint_state)
        rate.sleep()
        t += 1
        print(math.sin(t))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass