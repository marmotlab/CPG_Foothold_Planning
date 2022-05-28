#! /usr/bin/python

# foot hold correction based on the predicted foothold and cost map (potential map)

import rospy
import numpy as np
import rospy
import math
from std_msgs.msg import Float32MultiArray
from grid_map_msgs.msg import GridMap

global costmap_array, foothold, map_resolution, costmap_col_index, costmap_row_index, map_center_x, map_center_y, Flag1, Flag2

Flag1 = False
Flag2 = False


def costmap_callback(data):
    global costmap_array, map_resolution, costmap_col_index, costmap_row_index, map_center_x, map_center_y, Flag1
    Flag1 = True
    # print("subscribed")
    map_resolution = data.info.resolution
    costmap_col_index = np.array(data.data[0].layout.dim[0].size)  # map_dimension_x
    costmap_row_index = np.array(data.data[0].layout.dim[1].size)  # map_dimension_y
    cost_map_row = np.array(data.data[0].data)
    cost_map = np.reshape(cost_map_row,(costmap_row_index,costmap_col_index))
    costmap_array = np.flip(np.rot90(cost_map, 2), 0)

    # map_center_x = data.info.pose.position.x
    # map_center_y = data.info.pose.position.y


def foothold_callback(data):
    global foothold, Flag2
    Flag2 = True
    foothold = np.array(data.data)
    # print("subscribed_foothold")


rospy.init_node('foothold_correction')
leg_change_pub = rospy.Publisher('leg_need_change', Float32MultiArray, queue_size=1)
rospy.Subscriber('/cost_map', GridMap, costmap_callback, queue_size=1, buff_size=52428800)
rospy.Subscriber("/foothold_in_map", Float32MultiArray, foothold_callback, queue_size=1)
while not rospy.is_shutdown():
    # try:
    if Flag1 and Flag2:
            costmap_array_copy = np.copy(costmap_array)
            costmap_array_y_gra = np.gradient(costmap_array_copy)[1]
            costmap_array_x_gra = np.gradient(costmap_array_copy)[0]

            foothold_list = np.reshape(foothold[0:18], [6, 3]).T

            foot_num = []  # the leg's foothold which has been predicted
            for i in range(6):
                if foothold_list[1, i]:
                    foot_num.append(i)

            leg_need_to_change = [0] * 19  # leg_number + x dimension + y dimension + cpg['t']

            for leg in foot_num:
                foot_localmap_x = (costmap_row_index * map_resolution / 2 - (
                        foothold_list[1][leg])) // map_resolution
                foot_localmap_y = (costmap_col_index * map_resolution / 2 + (
                        foothold_list[0][leg])) // map_resolution
                costmap_value = costmap_array_copy[int(foot_localmap_x), int(foot_localmap_y)]
                if 0.01 < costmap_value:
                    leg_need_to_change[leg] = 1
                    # the gradient of the cost_map
                    gradient_y = costmap_array_y_gra[int(foot_localmap_x), int(foot_localmap_y)]
                    if gradient_y >= 0:
                        leg_need_to_change[leg + 6] = costmap_value
                    else:
                        leg_need_to_change[leg + 6] = -1 * costmap_value

                    gradient_x = costmap_array_x_gra[int(foot_localmap_x), int(foot_localmap_y)]
                    if gradient_x >= 0:
                        leg_need_to_change[leg + 12] = costmap_value
                    else:
                        leg_need_to_change[leg + 12] = -1 * costmap_value
                    print("leg_num: ", leg, "change value in x : ", leg_need_to_change[leg + 6], " in y : ", leg_need_to_change[leg + 12])

            leg_need_to_change[18] = foothold[18]  # topic cpg_time
            data_topublish = Float32MultiArray(data=leg_need_to_change)
            leg_change_pub.publish(data_topublish)
            print("foothold correcting....")
    else:
        print('finding costmap...')



