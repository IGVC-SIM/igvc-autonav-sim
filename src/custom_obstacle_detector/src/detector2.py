# #!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
# from nav_msgs.msg import OccupancyGrid
# import numpy as np

# # Define the limits of the 3D grid
# LIMX = 1000  # Example limit for x-axis
# LIMY = 1000  # Example limit for y-axis 
# LIMZ = 1000  # Example limit for z-axis

# # Initialize the 3D grid with zeros
# occupancy_grid = np.zeros((LIMX, LIMY, LIMZ), dtype=int)
# costmap2d = np.zeros((LIMX, LIMY), dtype=int)

# def point_cloud_callback(msg):
#     global occupancy_grid, costmap2d
#     # Reset the grid to zeros
#     occupancy_grid.fill(0)

#     # Convert PointCloud2 message to a list of tuples (x, y, z)
#     for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#         x, y, z = point[0], point[1], point[2]
        
#         # Convert coordinates to integers within the grid limits
#         # ix = int(np.clip(x, 0, LIMX-1))
#         # iy = int(np.clip(y, 0, LIMY-1))
#         # iz = int(np.clip(z, 0, LIMZ-1))
        
#         # Set the corresponding cell in the grid to 1
#         occupancy_grid[int(x), int(y), int(z)] = 1


#     # Update the 2D costmap based on the 3D occupancy grid
#     for x in range(0, LIMX):
#         for y in range(0, LIMY):
#             occupancy_column = occupancy_grid[x, y, :]
#             if 1 in occupancy_column:
#                 costmap2d[x, y] = 100
#             else:
#                 costmap2d[x, y] = 0

#     # Print the result (or process as needed)
#     # print("Costmap2D")
#     # print(costmap2d)
#     # print(costmap2d.shape)

#     # Publish the 2D costmap as an OccupancyGrid message
#     publish_costmap()


# def publish_costmap():
#     global costmap2d

#     occupancy_grid_msg = OccupancyGrid()
#     occupancy_grid_msg.header.stamp = rospy.Time.now()
#     occupancy_grid_msg.header.frame_id = "map"  # Set the appropriate frame ID

#     occupancy_grid_msg.info.resolution = 1.0  # Set the resolution of the grid (e.g., 1 meter per cell)
#     occupancy_grid_msg.info.width = costmap2d.shape[0]
#     occupancy_grid_msg.info.height = costmap2d.shape[1]

#     # Define the origin of the occupancy grid
#     occupancy_grid_msg.info.origin.position.x = 0.0
#     occupancy_grid_msg.info.origin.position.y = 0.0
#     occupancy_grid_msg.info.origin.position.z = 0.0
#     occupancy_grid_msg.info.origin.orientation.w = 1.0
#     occupancy_grid_msg.info.origin.orientation.x = 0.0
#     occupancy_grid_msg.info.origin.orientation.y = 0.0
#     occupancy_grid_msg.info.origin.orientation.z = 0.0

#     # Flatten the 2D costmap and assign it to the data field
#     occupancy_grid_msg.data = costmap2d.flatten().tolist()

#     # Publish the OccupancyGrid message
#     occupancy_grid_publisher.publish(occupancy_grid_msg)

# def main():
#     global occupancy_grid_publisher

#     rospy.init_node('point_cloud_subscriber', anonymous=True)

#     # Create a subscriber for the point cloud
#     rospy.Subscriber("/obstacle_detector/cloud_clusters", PointCloud2, point_cloud_callback)

#     # Create a publisher for the occupancy grid
#     occupancy_grid_publisher = rospy.Publisher("/costmap", OccupancyGrid, queue_size=10)

#     # Keep the node running until it is stopped
#     rospy.spin()

# if __name__ == '__main__':
#     main()


# # #!/usr/bin/env python3

# # import rospy
# # from sensor_msgs.msg import PointCloud2
# # import sensor_msgs.point_cloud2 as pc2
# # import ros_numpy
# # import numpy as np
# # import cv2
# # # from cv_bridge import CvBridge, CvBridgeError


# # def count_z_values(array, size):
# #     # Extract x, y, and z values from the array
# #     x_values = array[:, 0]
# #     y_values = array[:, 1]
# #     z_values = array[:, 2]

# #     # Find unique (x, y) pairs and their corresponding counts
# #     unique, counts = np.unique((x_values, y_values), axis=1, return_counts=True)

# #     # Initialize a 2D array to store the counts
# #     max_x = np.max(x_values)
# #     max_y = np.max(y_values)
# #     count_matrix = np.zeros((max_y + 1, max_x + 1), dtype=np.uint8)

# #     # Populate the count matrix with the counts of z values
# #     count_matrix[unique[1], unique[0]] = counts

# #     return count_matrix


# # def extract_rgb(rgb_float):
# #     # Convert the RGB float to an integer
# #     rgb_int = int(rgb_float)
# #     # Extract the red component (bits 16-23)
# #     r = (rgb_int >> 16) & 0x0000ff
# #     # Extract the green component (bits 8-15)
# #     g = (rgb_int >> 8) & 0x0000ff
# #     # Extract the blue component (bits 0-7)
# #     b = rgb_int & 0x0000ff
# #     return r, g, b


# # def callback_point_cloud(data):
# #     # Convert the ROS PointCloud2 message to a list of points
# #     # point_list = pc2.read_points(data, field_names=("x", "y", "z", "r","g","b"), skip_nans=True)
# #     # print([point for point in point_list])
# #     pc = ros_numpy.numpify(data)
# #     pc = ros_numpy.point_cloud2.split_rgb_field(pc)

# #     shape = pc.shape + (3, )

# #     points = np.zeros(shape) 
# #     points[..., 0] = np.round(pc['z'],0)
# #     points[..., 1] = np.round(pc['x'],0)
# #     points[..., 2] = -np.round(pc['y'],0)
# #     rgb = np.zeros(shape)
# #     rgb[..., 0] = pc['r']
# #     rgb[..., 1] = pc['g']
# #     rgb[..., 2] = pc['b']
    

#     # xmatching_indexes = np.isin(points[..., 0], x_values)
#     # ymatching_indexes = np.isin(points[..., 1], y_values)

#     # min_z = np.min(points[:, 2])

#     # bottom_layer_indexes = np.where(points[:, 2] == min_z)

#     # sorted_pointmap = points[np.lexsort((points[:,2], points[:,1], points[:,0]))]

#     # # Create a boolean mask where True indicates the first occurrence of a new (x, y) pair
#     # unique_xy_mask = np.ones(sorted_pointmap.shape[0], dtype=bool)
#     # unique_xy_mask[1:] = (sorted_pointmap[1:, 0] != sorted_pointmap[:-1, 0]) | (sorted_pointmap[1:, 1] != sorted_pointmap[:-1, 1])

#     # # Select only the rows that correspond to the minimum z-value for each (x, y) pair
#     # bottom_layer = sorted_pointmap[unique_xy_mask]


#     # sorted_pointmap = points[np.lexsort((white_points[:,2], white_points[:,1], white_points[:,0]))]

#     # # Create a boolean mask where True indicates the first occurrence of a new (x, y) pair
#     # unique_xy_mask = np.ones(sorted_pointmap.shape[0], dtype=bool)
#     # unique_xy_mask[1:] = (sorted_pointmap[1:, 0] != sorted_pointmap[:-1, 0]) | (sorted_pointmap[1:, 1] != sorted_pointmap[:-1, 1])

#     # # Select only the rows that correspond to the minimum z-value for each (x, y) pair
#     # white_bottom_layer = sorted_pointmap[unique_xy_mask]

#     # bottom_layer = points[bottom_layer_indexes]
#     # print(points.shape)
#     # print(bottom_layer.shape)

#     # dtype = np.dtype((np.void, points.dtype.itemsize * points.shape[1]))
#     # array1_view = points.view(dtype)
#     # array2_view = bottom_layer.view(dtype)

#     # # Use np.in1d to find the common elements
#     # bottom_layer_indexes = np.in1d(array1_view, array2_view)
#     # # bottom_layer_colormap = colormap[bottom_layer_indexes]

#     # print(bottom_layer.shape)
#     # print(bottom_layer_colormap.shape)

#     # white_bottom_layer = bottom_layer[np.where((bottom_layer_colormap[..., 0] > threshold) & 
#     #                         (bottom_layer_colormap[..., 1] > threshold) & 
#     #                         (bottom_layer_colormap[..., 2] > threshold))]
#     # print(f"{len(white_bottom_layer)}/{len(bottom_layer)} of points are white")
    
#     # Step 1: Remove z component to get (x, y) positions
#     # xy_positions = points[:, :2]

#     # # xy_positions = xy_positions.astype(np.uint8)

#     # # Step 2: Determine the size of the image
#     # max_x = np.max(xy_positions[:, 0])
#     # max_y = np.max(xy_positions[:, 1])

#     # xy_positions = xy_positions/np.array([max_x, max_y])
#     # print(xy_positions)
#     # xy_positions = xy_positions*np.array([int(len(xy_positions[:, 0]/10)), int(len(xy_positions[:, 1])/10)])

#     # xy_positions = xy_positions.astype(np.uint8)

#     # # Create a black image with size based on max (x, y) positions
#     # image = np.zeros((int(len(xy_positions[:, 1])/10)+1, int(len(xy_positions[:, 0])/10)+1), dtype=np.uint8)

#     # # Set the positions in the image to white
#     # image[xy_positions[:, 1], xy_positions[:, 0]] = 255

#     # bridge = CvBridge()
#     # try:
#     #     # Convert the ROS Image message to a format OpenCV can use
#     #     cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#     # except CvBridgeError as e:
#     #     print(e)
    
#     # Display the image
#     max_x = np.max(points[:, 0])
#     max_y = np.max(points[:, 1])
#     points = points/np.array([max_x, max_y, 1])
#     print(points)
#     points = points*np.array([int(len(points[:, 0])), int(len(points[:, 1])), 1])
#     points = points.astype(np.uint8)
#     image = count_z_values(points, [len(points[:,1]), len(points[:,0])])
#     print(image)
#     print(image.shape)
#     image = image/np.max(image.flatten())
#     image = image*255
#     cv2.imshow("Subscribed Image", image)
#     cv2.waitKey(1)
#     # Step 3: Display the image using OpenCV

#     # for point in point_list:
#     #     x, y, z, rgb = point
#     #     r, g, b = extract_rgb(rgb)
#     #     rospy.loginfo("x: %f, y: %f, z: %f, r: %d, g: %d, b: %d", x, y, z, r, g, b)

# def main():
#     rospy.init_node('point_cloud_subscriber', anonymous=True)
#     rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, callback_point_cloud)
#     rospy.spin()

# if __name__ == '__main__':
#     main()



# # import rospy
# # import cv2
# # import numpy as np
# # from sensor_msgs.msg import Image, PointCloud2
# # from cv_bridge import CvBridge
# # from sensor_msgs.msg import CameraInfo
# # import sensor_msgs.point_cloud2 as pc2
# # import pcl_msgs
# # from pcl_msgs.msg import PointIndices
# # from geometry_msgs.msg import Point32
# # from std_msgs.msg import Header

# # class PointCloudProcessor:
# #     def __init__(self):
# #         self.bridge = CvBridge()
# #         self.rgb_sub = rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, self.rgb_callback)
# #         self.depth_sub = rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", Image, self.depth_callback)
# #         self.camera_info_sub = rospy.Subscriber("/zed2i/zed_node/rgb/camera_info", CameraInfo, self.cam_info_callback)
# #         self.cloud_sub = rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, self.cloud_callback)
# #         self.cloud_pub = rospy.Publisher("/marked_point_cloud", PointCloud2, queue_size=10)
# #         self.cam_matrix = None
# #         self.rgb_image = None
# #         self.depth_image = None
# #         self.cloud = None
# #         self.cloud_msg = None

# #     def rgb_callback(self, rgb):
# #         self.rgb_image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding="passthrough")

# #     def depth_callback(self, depth):
# #         self.depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")

# #     def cam_info_callback(self, cam_info):
# #         self.cam_matrix = np.array(cam_info.K).reshape(3, 3)

# #     def cloud_callback(self, cloud):
# #         self.cloud = cloud
# #         if self.rgb_image is not None and self.depth_image is not None and self.cam_matrix is not None:
# #             self.process_cloud()

# #     def process_cloud(self):
# #         hsv_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
# #         orange_mask = cv2.inRange(hsv_image, (5, 50, 50), (15, 255, 255))  # Orange color range
# #         white_mask = cv2.inRange(hsv_image, (0, 0, 200), (180, 30, 255))   # White color range
# #         combined_mask = cv2.bitwise_or(orange_mask, white_mask)

# #         points = list(pc2.read_points(self.cloud, skip_nans=True, field_names=("x", "y", "z")))

# #         marked_points = []
# #         column_indices = []

# #         for p in points:
# #             x, y, z = p
# #             if np.isnan(x) or np.isnan(y) or np.isnan(z):
# #                 continue
# #             u = int((x * self.cam_matrix[0, 0] / z) + self.cam_matrix[0, 2])
# #             v = int((y * self.cam_matrix[1, 1] / z) + self.cam_matrix[1, 2])
# #             if u < 0 or u >= self.rgb_image.shape[1] or v < 0 or v >= self.rgb_image.shape[0]:
# #                 continue
# #             if combined_mask[v, u] > 0:
# #                 column_indices.append(u)

# #         if column_indices:
# #             min_index = min(column_indices)
# #             max_index = max(column_indices)

# #             for i, p in enumerate(points):
# #                 x, y, z = p
# #                 if min_index <= int((x * self.cam_matrix[0, 0] / z) + self.cam_matrix[0, 2]) <= max_index:
# #                     marked_points.append(Point32(x=x, y=y, z=z))

# #             marked_cloud_msg = pc2.create_cloud_xyz32(self.cloud.header, marked_points)
# #             self.cloud_pub.publish(marked_cloud_msg)

# # if __name__ == "__main__":
# #     rospy.init_node("point_cloud_processor")
# #     processor = PointCloudProcessor()
# #     rospy.spin()
