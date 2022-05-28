import numpy as np
from math import sqrt


def groundIK(cpg, dist, indicies):
    # base = 1:3:18;
    # shoulders = 2:3:18;
    # elbows = 3:3:18;
    # print(indicies)

    # q1 = cpg['legs'][0,3*index]
    # q2 = cpg['legs'][0,3*index+1]
    # q3 = cpg['legs'][0,3*index+2]

    # elbowOut = q3;

    positions = cpg['xmk'].getLegPositions(cpg['legs'])
    # print(cpg["legs"][0, 0:3])
    # print(np.reshape(cpg['legs'],(3,6)))


    for index in indicies:
        reqd_position = positions[:, index]

        inter_FK = cpg['xmk'].getLegFrames(cpg['legs'])

        ee_Frame = np.squeeze(inter_FK[index, 5, :, :])            # end effector frame wrt. base frame
        homeFrame = np.squeeze(inter_FK[index, 2, :, :])           # shoulder actuator frame wrt. base frame


        varFrame = np.squeeze(inter_FK[index, 3, :, :])            # shoulder link frame  wrt. base frame
        # if index==3:
        #     print("base in body",np.squeeze(inter_FK[index, 0, :, 3]))
        #     print(np.squeeze(inter_FK[index, 1, :, 3]))
        #     print(np.squeeze(inter_FK[index, 2, :, 3]))
        #     print(np.squeeze(inter_FK[index, 3, :, 3]))
        #     print(np.squeeze(inter_FK[index, 4, :, 3]))
        #     print("foot in body",np.squeeze(inter_FK[index, 5, :, 3]))    # foot
        #     print(".................")


        reqd_dist = dist[index]

        ee_home = np.dot(np.linalg.pinv(homeFrame), ee_Frame)      # end effector expressed in shoulder actuator frame
        z0 = ee_home[2, 3]           # 0.08 fixed value :end effector's point in z axis wrt. shoulder actuator frame

        ee_varFrame = np.dot(np.linalg.pinv(varFrame), ee_Frame[:, 3])   # end effector expressed in shoulder link frame

        fixed_dist = np.linalg.norm(ee_varFrame[0:3])   #  fixed value : the distanc between end effector frame and shoulder link frame

        last_fixpt = np.dot(np.linalg.pinv(homeFrame), varFrame[:, 3])     # fix value : shoulder link frame expressed in shoulder actuator frame


        x1 = last_fixpt[0]   # 0.325
        y1 = last_fixpt[1]   # 0
        z1 = last_fixpt[2]   # 0

        d = fixed_dist ** 2 - (z1 - z0) ** 2  # fix value : the elbow link's length

        # the y_axis_vector (base_frame) express in shoulder actuator frame
        tf_yvec = np.dot(np.linalg.pinv(homeFrame), np.array([[0, 1, 0, 0]]).T)

        # desired distance between the desired foot trajectory and shoulder actuator frame
        new_dist = reqd_dist - homeFrame[1, 3]


        k = (new_dist - tf_yvec[2] * z0) / tf_yvec[0]
        b = tf_yvec[1] / tf_yvec[0]


        x01 = (k + b * (b * x1 - y1 + sqrt((1 + b ** 2) * d - (-k + x1 + b * y1) ** 2))) / (1 + b ** 2)  # error here
        y01 = (k - x01) / b

        x02 = (k + b * (b * x1 - y1 - sqrt((1 + b ** 2) * d - (-k + x1 + b * y1) ** 2))) / (1 + b ** 2)
        y02 = (k - x02) / b

        final_pt1 = np.dot(homeFrame, np.array([[x01, y01, z0, 1]]).T)
        final_pt2 = np.dot(homeFrame, np.array([[x02, y02, z0, 1]]).T)

        if final_pt1[2] > final_pt2[2]:
            final_pt = final_pt2[0:3]
        else:
            final_pt = final_pt1[0:3]

        # print('this is DD',homeFrame)

        positions[:, index] = final_pt[:, 0]

    angs = cpg['xmk'].getLegIK(positions)

    return angs



def groundIK_leg(cpg, dist, index):
    # base = 1:3:18;
    # shoulders = 2:3:18;
    # elbows = 3:3:18;
    # print(indicies)

    # q1 = cpg['legs'][0,3*index]
    # q2 = cpg['legs'][0,3*index+1]
    # q3 = cpg['legs'][0,3*index+2]

    # elbowOut = q3;

    # positions = cpg['xmk'].getLegPositions_leg(cpg['legs'])
    # print(cpg["legs"][0, 0:3])

    # reqd_position = positions[:, index]

    inter_FK = cpg['xmk'].getLegFrames_leg(index, cpg['legs'])

    ee_Frame = np.squeeze(inter_FK[0, 5, :, :])            # end effector frame wrt. base frame
    homeFrame = np.squeeze(inter_FK[0, 2, :, :])           # shoulder actuator frame wrt. base frame


    varFrame = np.squeeze(inter_FK[0, 3, :, :])            # shoulder link frame  wrt. base frame
    # if index==3:
    #     print("base in body",np.squeeze(inter_FK[index, 0, :, 3]))
    #     print(np.squeeze(inter_FK[index, 1, :, 3]))
    #     print(np.squeeze(inter_FK[index, 2, :, 3]))
    #     print(np.squeeze(inter_FK[index, 3, :, 3]))
    #     print(np.squeeze(inter_FK[index, 4, :, 3]))
    #     print("foot in body",np.squeeze(inter_FK[index, 5, :, 3]))    # foot
    #     print(".................")


    reqd_dist = dist[index]

    ee_home = np.dot(np.linalg.pinv(homeFrame), ee_Frame)      # end effector expressed in shoulder actuator frame
    z0 = ee_home[2, 3]           # 0.08 fixed value :end effector's point in z axis wrt. shoulder actuator frame

    ee_varFrame = np.dot(np.linalg.pinv(varFrame), ee_Frame[:, 3])   # end effector expressed in shoulder link frame

    fixed_dist = np.linalg.norm(ee_varFrame[0:3])   #  fixed value : the distanc between end effector frame and shoulder link frame

    last_fixpt = np.dot(np.linalg.pinv(homeFrame), varFrame[:, 3])     # fix value : shoulder link frame expressed in shoulder actuator frame


    x1 = last_fixpt[0]   # 0.325
    y1 = last_fixpt[1]   # 0
    z1 = last_fixpt[2]   # 0

    d = fixed_dist ** 2 - (z1 - z0) ** 2  # fix value : the elbow link's length

    # the y_axis_vector (base_frame) express in shoulder actuator frame
    tf_yvec = np.dot(np.linalg.pinv(homeFrame), np.array([[0, 1, 0, 0]]).T)

    # desired distance between the desired foot trajectory and shoulder actuator frame
    new_dist = reqd_dist - homeFrame[1, 3]


    k = (new_dist - tf_yvec[2] * z0) / tf_yvec[0]
    b = tf_yvec[1] / tf_yvec[0]


    x01 = (k + b * (b * x1 - y1 + sqrt((1 + b ** 2) * d - (-k + x1 + b * y1) ** 2))) / (1 + b ** 2)  # error here
    y01 = (k - x01) / b

    x02 = (k + b * (b * x1 - y1 - sqrt((1 + b ** 2) * d - (-k + x1 + b * y1) ** 2))) / (1 + b ** 2)
    y02 = (k - x02) / b

    final_pt1 = np.dot(homeFrame, np.array([[x01, y01, z0, 1]]).T)
    final_pt2 = np.dot(homeFrame, np.array([[x02, y02, z0, 1]]).T)

    if final_pt1[2] > final_pt2[2]:
        final_pt = final_pt2[0:3]
    else:
        final_pt = final_pt1[0:3]

    # print('this is DD',homeFrame)
    positions = np.zeros((3, 6))

    positions[:, index] = final_pt[:, 0]

    angs = cpg['xmk'].getLegIK_leg(index, positions)
    # angs = cpg['xmk'].getLegIK(positions)

    return angs