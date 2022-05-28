#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import tf_conversions
import geometry_msgs.msg
import rosbag
import numpy as np

def sendH(H,br,frameName,worldFrame):
    scale, shear, rpy_angles, translation_vector, perspective = tf_conversions.transformations.decompose_matrix(H)

    q = tf_conversions.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])

    print(frameName)
    br.sendTransform(translation_vector, q, rospy.Time.now(),frameName,worldFrame)

def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
                       
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R



def transformer(frameName,worldFrame, firstFrame,HFirst):
    retVal = False
    try:
        (trans,rot) = listener.lookupTransform(worldFrame, frameName,rospy.Time(0))
        retVal = True
        if frameName == 'Robot':
            qNew = tuple([-rot[1],rot[0],rot[2],rot[3]])
            tNew = tuple([-trans[1],trans[0],trans[2]])
        else:
            qNew = tuple([rot[0],rot[1],rot[2],rot[3]])
            tNew = tuple([trans[0],trans[1],trans[2]])


        if firstFrame:
            thetas = tf_conversions.transformations.euler_from_quaternion(qNew)
            rotM = eulerAnglesToRotationMatrix(thetas)
            if frameName != 'camera1_pose_frame':
                HFirst[0:3,0:3] = rotM.T
                HFirst[0:3,3] = np.dot(rotM.T,-np.array(tNew))
            else:
                HFirst[0:3,0:3] = rotM.T
                HFirst[0:3,3] = -np.array(tNew)
            firstFrame = False
            retVal = True


        
        thetas = tf_conversions.transformations.euler_from_quaternion(qNew)        
        rotM = eulerAnglesToRotationMatrix(thetas)
        HFrame = np.identity(4)
        HFrame[0:3,0:3] = rotM
        HFrame[0:3,3] = tNew

        

        if frameName != 'camera1_pose_frame':
            FinalH = np.dot(HFirst,HFrame)
        else:
            HNew = np.identity(4)
            HNew[0:3,3] = np.dot(rotM.T,HFirst[0:3,3])
            HNew[0:3,0:3] = HFirst[0:3,0:3] 
            FinalH = np.dot(HFrame,HNew)
        
        br = tf.TransformBroadcaster()
        sendH(FinalH,br,frameName + 'trans',worldFrame)

        return HFirst,firstFrame, retVal


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return HFirst,firstFrame,retVal





if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')
    listener = tf.TransformListener()
    firstFrameKin = True
    firstFrameOpti = True
    firstFrameCam = True
    rate = rospy.Rate(100.0)
    HFirstOpti = np.identity(4)
    HFirstKin = np.identity(4)
    HFirstCam = np.identity(4)
    HCam = np.identity(4)
    while not rospy.is_shutdown():
        HFirstKin,firstFrameKin, retVal = transformer('base_link','world',firstFrameKin,HFirstKin)
        #retVal = True
        if retVal:
            print('broadcasting')

            HFirstOpti,firstFrameOpti, retVal = transformer('Robot','world',firstFrameOpti,HFirstOpti)

            HFirstCam,firstFrameCam, retVal = transformer('camera1_pose_frame','camera1_spatial',firstFrameCam,HFirstCam)


            br = tf.TransformBroadcaster()
            sendH(HCam,br,'camera1_spatial','world')

            rate.sleep()
