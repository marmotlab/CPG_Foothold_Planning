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

message = ''
started = False
started_foothold = False

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
    # TODO: add the suitable value of the variance
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
    message_tem = grid_msg
    grid = grid_msg.data
    a = grid[0].data
    frame = grid_msg.basic_layers
    # print("publish_costmap!!!")
    # print(frame)

    map_resolution = grid_msg.info.resolution
    # get grid map dimensions
    array_dimension_x = grid[0].layout.dim[0].size   #80*80
    array_dimension_y = grid[0].layout.dim[1].size   #80*80
    # print(a[0])

    map_center_x = grid_msg.info.pose.position.x
    map_center_y = grid_msg.info.pose.position.y

    # convert gridmap data in to numpy array and reshape to 2D
    b = np.array(a)
    c = b.reshape(array_dimension_x, array_dimension_y)

    hm_array_zero = np.nan_to_num(c)
    # np.save('elevation_map', hm_array_zero)


    # extracts a subarray
    new_hm_array_y_gra = np.gradient(hm_array_zero)[1]
    new_hm_array_x_gra = np.gradient(hm_array_zero)[0]
    new_hm_array_2d_gra = np.sqrt(np.power(new_hm_array_y_gra, 2) + np.power(new_hm_array_x_gra, 2))
    temp_map = new_hm_array_2d_gra
    cost_map = np.zeros([grid[0].layout.dim[0].size, grid[0].layout.dim[1].size])
    #[first_row:last_row, first_col:last_col]
    for i in range(array_dimension_x):
        for j in range(array_dimension_y):
            if abs(temp_map[i, j]) > 0.003:
                cost_map = addGaussian(cost_map, 1/(abs(temp_map[i,j]*30)), np.array([i,j]))
    cost_map_Gau = np.flip(np.rot90(cost_map, 1), 0)*0.1
    costmap_xy = tuple(cost_map_Gau.reshape(1, -1)[0])

    message_tem.data[0].data = costmap_xy
    message = message_tem

    costmap_array = np.flip(np.rot90(cost_map_Gau, 2), 0)

    if not started:
        started = True


def foothold_callback(data):
    global foothold_map, started_foothold

    foothold_map = np.array(data.data)   # 1*18 array

    if not started_foothold:
        started_foothold = True


rospy.init_node('costmap_generator')

if __name__ == '__main__':
    pub = rospy.Publisher('cost_map', GridMap, queue_size=1)
    rospy.Subscriber('/elevation_map_matrix', GridMap, grid_callback,queue_size=1,buff_size=52428800)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            if started:
                pub.publish(message)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
