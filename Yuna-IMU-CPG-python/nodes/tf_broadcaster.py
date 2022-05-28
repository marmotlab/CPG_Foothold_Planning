#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import hebi
import numpy as np
import quaternion
import copy
#ohh


from xmonster_cpg import setup


def getTransform(H):
    scale, shear, rpy_angles, translation_vector, perspective = tf_conversions.transformations.decompose_matrix(H)

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = translation_vector[0]
    t.transform.translation.y = translation_vector[1]
    t.transform.translation.z = translation_vector[2]

    q = tf_conversions.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    return t


if __name__ == '__main__':

    rospy.init_node('tf_publisher')
    br = tf2_ros.TransformBroadcaster()

    xmk, CF, imu , hexapod, fbk_imu, fbk_hp = setup.setup_xmonster()
    
    names = ['base','shoulder','elbow','feet']
    pos = np.reshape(fbk_hp.position, (1, 18))
    frames = xmk.getHexapodFrames(pos)
    feet = frames[3]
    xmk.stanceFeet = feet

    rate = rospy.Rate(200.0)
    


    while not rospy.is_shutdown() and (fbk_hp != None):


        #Update Raw Data
        pos = np.reshape(fbk_hp.position, (1, 18))
        torques = np.reshape(fbk_hp.position, (1, 18))

        frames = xmk.getHexapodFrames(pos)
        feet = frames[3]
        legTorques = xmk.getLegTorques(pos,torques)
        contactLegs = xmk.getContactLegs(legTorques)
        xmk.updateBaseFrame(contactLegs,feet)

        for leg in range(6):
            for joint in range(4):
                frame = frames[joint]

                H = frame[leg]
                t = getTransform(H)
                t.header.frame_id = "robot"
                t.child_frame_id = names[joint] + str(leg+1)
                br.sendTransform(t)

        t = getTransform(xmk.robot_base)
        t.header.frame_id = "world"
        t.child_frame_id = "robot"

        br.sendTransform(t)

        print("legTorques")
        sum = 0;
        for torque in fbk_hp.effort:
            sum += abs(torque)
        print(sum)
        rate.sleep()
        fbk_hp = hexapod.get_next_feedback(reuse_fbk=fbk_hp)

