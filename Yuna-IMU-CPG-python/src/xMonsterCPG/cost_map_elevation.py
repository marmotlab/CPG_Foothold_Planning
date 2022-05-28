#!/usr/bin/env python

import rospy
import time
from grid_map_msgs.msg import GridMap
from grid_map_msgs.msg import GridMapInfo
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf

global foothold, foothold_Flag, foot_localmap_x, foot_localmap_y

message = ''
started = False
foothold_Flag = False
foot_localmap_x = 50
foot_localmap_y = 50

def addGaussian(self, variance, gaussian_mean):
    """
    1. generate some random matrix represent the mean and variance of the distribution
    2. create the distribution in a 20x20 matrix
    3. for now the distribution contains decimal and the sum of the whole matrix of each distribution is 1
    4. the position of the distribution is the mean in a list (row,col)
    """
    # gaussian_mean = self.shape[0] * np.random.rand(1, 2)[0]
    gaussian_var = np.zeros((2, 2))
    # gaussian_var[([0, 1], [0, 1])] = 0.2 * self.shape[0] * np.random.rand(1, 2)[0]
    gaussian_var[([0, 1], [0, 1])] = variance * np.array([1, 1]) #* np.random.rand(1, 2)[0]
    # if variance > 10:
    #     gaussian_var[([0, 1], [0, 1])] = 10 + variance * np.random.rand(1, 2)[0]
    row_mat, col_mat = np.meshgrid(np.linspace(0, self.shape[0] - 1, self.shape[0]),
                                   np.linspace(0, self.shape[1] - 1, self.shape[1]))
    SigmaX = np.sqrt(gaussian_var[0][0])
    SigmaY = np.sqrt(gaussian_var[1][1])
    Covariance = gaussian_var[0][1]
    r = Covariance / (SigmaX * SigmaY)
    coefficients = 1 / (2 * math.pi * SigmaX * SigmaY * np.sqrt(1 - math.pow(r, 2)))
    p1 = -1 / (2 * (1 - math.pow(r, 2)))
    px = np.power(row_mat - gaussian_mean[0], 2) / gaussian_var[0][0]
    py = np.power(col_mat - gaussian_mean[1], 2) / gaussian_var[1][1]
    pxy = 2 * r * (row_mat - gaussian_mean[0]) * (col_mat - gaussian_mean[1]) / (SigmaX * SigmaY)
    distribution_matrix = coefficients * np.exp(p1 * (px - pxy + py)) + self
    return distribution_matrix


def grid_callback(grid_msg):
    # subscribe to the gridmap data
    global message, started, costmap_array, array_dimension_x, array_dimension_y, map_resolution, map_center_x, map_center_y, costmap_array_y_gra, costmap_array_x_gra
    global foot_localmap_x, foot_localmap_y, map_data, message_tem
    message_tem = grid_msg
    grid = grid_msg.data
    map_data = grid[0].data
    #frame = grid_msg.basic_layers   # 'elevation', 'upper_bound', 'lower_bound'



    map_resolution = grid_msg.info.resolution
    # get grid map dimensions
    array_dimension_x = grid[0].layout.dim[0].size   #80*80
    array_dimension_y = grid[0].layout.dim[1].size   #80*80

    # map_center_x = grid_msg.info.pose.position.x
    # map_center_y = grid_msg.info.pose.position.y




    if not started:
        started = True


def foothold_callback(data):
    global foothold, foothold_Flag
    foothold_Flag = True
    foothold = np.array(data.data)



rospy.init_node('costmap_generator')



if __name__ == '__main__':
    listener = tf.TransformListener()
    pub = rospy.Publisher('cost_map', GridMap, queue_size=1)
    # leg_change_pub = rospy.Publisher('leg_need_change', Float32MultiArray, queue_size=1)
    rospy.Subscriber('/elevation_map_matrix', GridMap, grid_callback,queue_size=1,buff_size=52428800)
    rospy.Subscriber("/foothold_in_map", Float32MultiArray, foothold_callback, queue_size=1)
    # rospy.Subscriber("/foothold_map", Float32MultiArray, foothold_callback, queue_size=1)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # make the array from same message

        try:
            if started and foothold_Flag:
                # convert grid map data in to numpy array and reshape to 2D
                b = np.array(map_data)
                c = b.reshape(array_dimension_x, array_dimension_y)
                elevation_matrix = np.nan_to_num(c)
                map_array = np.flip(np.rot90(elevation_matrix, 2), 0)
                # np.save('elevation_map', elevation_matrix)
                # print("saved!!!")
                # (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                # print("trans: ", trans[2])

                # calculate the 2D gradient
                # new_hm_array_y_gra = np.gradient(elevation_matrix)[1]
                # new_hm_array_x_gra = np.gradient(elevation_matrix)[0]
                # new_hm_array_2d_gra = new_hm_array_y_gra + new_hm_array_x_gra
                # new_hm_array_2d_gra = np.sqrt(np.power(new_hm_array_y_gra, 2) + np.power(new_hm_array_x_gra, 2))

                temp_map = map_array

                # creat an empty costmap
                cost_map = np.zeros([array_dimension_x, array_dimension_y])
                # [first_row:last_row, first_col:last_col]
                foothold_list = np.reshape(foothold[0:18], [6, 3]).T

                foot_num = []  # the leg's foothold which has been predicted
                for i in range(6):
                    if foothold_list[1, i]:
                        foot_num.append(i)

                for leg in foot_num:
                    foot_localmap_x = (array_dimension_y * map_resolution / 2 - (
                        foothold_list[1][leg])) // map_resolution
                    foot_localmap_y = (array_dimension_x * map_resolution / 2 + (
                        foothold_list[0][leg])) // map_resolution

                    new_hm_array = elevation_matrix  # [first_row:last_row, first_col:last_col]
                    for i in range(int(foot_localmap_x-5), int(foot_localmap_x + 5)):
                        for j in range(int(foot_localmap_y-5), int(foot_localmap_y +5)):
                            # print(cost_map[40,40])

                            # if abs(temp_map[i, j]) > 0.03:
                            # cost_map = addGaussian(cost_map, 1/(abs(temp_map[i,j]*20)), np.array([i,j]))
                                cost_map = addGaussian(cost_map, 2, np.array([i, j]))

                # cost_map = addGaussian(cost_map, abs(temp_map[0][0])*10000, np.array([0, 0]))
                # print(abs(temp_map[0][0]))
                # cost_map_Gau = np.flip(np.rot90(cost_map, 1), 0) * 0.3
                cost_map_Gau = np.rot90(cost_map, -1) * 0.3
                costmap_xy = tuple(cost_map_Gau.reshape(1, -1)[0])

                message_tem.data[0].data = costmap_xy
                message = message_tem

                costmap_array = np.flip(np.rot90(cost_map_Gau, 2), 0)
                # costmap_array_y_gra = np.gradient(costmap_array)[1]
                # costmap_array_x_gra = np.gradient(costmap_array)[0]
                # np.save('ele_cost_map_saved', costmap_array)
                # print("saved!!!")
                pub.publish(message)
                rate.sleep()

                # costmap_array_copy = np.copy(costmap_array)
                # costmap_array_x_gra_copy = np.copy(costmap_array_x_gra)
                # costmap_array_y_gra_copy = np.copy(costmap_array_y_gra)
                # print('published!')
                # if started_foothold:
                #
                #     foothold_list = np.reshape(foothold_map[0:18], [6, 3]).T
                #     foot_num = []    # the leg's foothold which has been predicted
                #     for i in range(6):
                #         if foothold_list[0, i]:
                #             foot_num.append(i)
                #     leg_need_to_change = [0]*19    #  leg_number + x dimension + y dimension + cpg['t']
                #     for leg in foot_num:
                #
                #         foot_localmap_x = (array_dimension_y * map_resolution / 2 - (
                #                     foothold_list[1][leg] - map_center_y)) // map_resolution
                #         foot_localmap_y = (array_dimension_x * map_resolution / 2 + (
                #                     foothold_list[0][leg] - map_center_x)) // map_resolution
                #
                #         if 0.01 <costmap_array_copy[int(foot_localmap_x), int(foot_localmap_y)] < 0.14 :
                #             # np.set_printoptions(threshold=np.inf)
                #             leg_need_to_change[leg] = 1
                #             # the gradient of the cost_map
                #             gradient_y = costmap_array_y_gra_copy[int(foot_localmap_x), int(foot_localmap_y)]
                #             leg_need_to_change[leg+6] = gradient_y
                #             gradient_x = costmap_array_x_gra_copy[int(foot_localmap_x), int(foot_localmap_y)]
                #             leg_need_to_change[leg+12] = gradient_x
                #
                #     leg_need_to_change[18] = foothold_map[18]
                #     data_topublish = Float32MultiArray(data=leg_need_to_change)
                #     leg_change_pub.publish(data_topublish)


        except rospy.ROSInterruptException:
            print("cost map building ...")
