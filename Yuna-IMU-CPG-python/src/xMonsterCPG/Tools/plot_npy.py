import numpy as np
import matplotlib.pyplot as plt
from skimage.transform import resize as resize

# data = np.load('/home/marmot/elevation_ws/new_hm_array_2d_gra.npy')
data = np.load('/home/marmot/elevation_ws/elevation_map.npy')
data = np.flip(np.rot90(data, 2), 0)
costmap_array_y_gra = np.gradient(data)[1]
costmap_array_x_gra = np.gradient(data)[0]
costmap_array_xy_gra = np.sqrt(np.power(costmap_array_y_gra,2)+np.power(costmap_array_x_gra,2))
elevation_map = np.flip(np.rot90(data, 1), 0)

np.set_printoptions(threshold=np.inf)
plt.imshow(data)
plt.show()
plt.imshow(costmap_array_y_gra)
plt.show()
plt.imshow(costmap_array_x_gra)

plt.show()
plt.imshow(costmap_array_xy_gra)

plt.show()
# print(data)