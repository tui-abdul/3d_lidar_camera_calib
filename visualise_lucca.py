import rclpy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from rclpy.node import Node
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from rclpy import  qos
import message_filters
from sensor_msgs_py.point_cloud2 import read_points
import matplotlib.pyplot as plt
#from pylon_test_2 import execute_code

class camera_publisher(Node):

    def __init__(self):
        super().__init__('publisher')
    
        with open("param.yaml","r") as file_handler:
            load_data = yaml.safe_load(file_handler)
        with open(load_data["camera_intrinsic_param"] , "r") as file_handle:
        
        
            self.calib_data = yaml.safe_load(file_handle)
        
        matrix_coefficients =    self.calib_data["camera_matrix"]["data"]
        distortion_coefficients = self.calib_data["distortion_coefficients"]["data"]
        self.matrix_coefficients = np.array(matrix_coefficients).reshape(3,3)
        self.distortion_coefficients = np.array(distortion_coefficients)
    

        image_color = load_data["camera_topic"]  
        ouster = load_data["lidar_topic"]
                # Subscribe to topics
        image_sub = message_filters.Subscriber(self,Image,image_color)
        ouster = message_filters.Subscriber(self, PointCloud2,ouster,qos_profile= qos.qos_profile_sensor_data)

        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub, ouster], queue_size=15, slop=0.05, allow_headerless=True)
        ats.registerCallback(self.callback)
        self.bridge = CvBridge()
   

        self.tvec = np.array([  -0.01180592, -0.07347713, 0.11045512],dtype=np.float64).reshape((3,1)) # lucca 2
        self.rvec = np.array([  1.236434  ,-1.21476398, 1.18096936] ,dtype=np.float64).reshape((3,1)) # lucca 2

        print("initialization done")

    def fix_data_type(self, array):
        points_array_ring = np.zeros((65536, 4))
        for i, point in enumerate(array):
            x,y,z, intensity = point
            points_array_ring[i,0] = x
            points_array_ring[i,1] = y
            points_array_ring[i,2] = z
            points_array_ring[i,3] = intensity 
        #print(points_array_ring)
        return points_array_ring
    
    def trasnformation(self,pc_as_numpy_array):
        t_mat =np.array([
                [-1, 0, 0, 0], 
                [0, -1, 0, 0], 
                [0, 0, 1, 0.038195], #make it zero as well
                [0, 0, 0, 1]
            ])
        column_of_ones = np.ones((pc_as_numpy_array.shape[0] , 1))
        
        result_array = np.hstack((pc_as_numpy_array, column_of_ones))
        transformed_point_3d = np.dot(t_mat, result_array.T)
        #print('t_point',transformed_point_3d.shape)
        return transformed_point_3d.T
    
    

    def filter_points_within_yaw(self,lidar_points, yaw_left, yaw_right):
        """
        Filters LiDAR points within a specified yaw angle range.
        
        Parameters:
        lidar_points (np.ndarray): Nx3 array of LiDAR points.
        yaw_left (float): Yaw angle to the left in degrees.
        yaw_right (float): Yaw angle to the right in degrees.
        
        Returns:
        np.ndarray: Boolean mask indicating which points are within the specified yaw angle range.
        """
        yaw_left_rad = np.deg2rad(yaw_left)  # Convert left yaw angle to radians
        yaw_right_rad = np.deg2rad(yaw_right)  # Convert right yaw angle to radians
        angles = np.arctan2(lidar_points[:, 1], lidar_points[:, 0])
        mask = (angles >= -yaw_right_rad) & (angles <= yaw_left_rad)
        return mask

    

    
    def callback(self, image_msg,lidar_msg):
        print('new msg arrived')
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="mono8")
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        undistorted_image = cv2.undistort(image, self.matrix_coefficients,self.distortion_coefficients)
        points_gen =  read_points(lidar_msg, field_names=('x','y','z','intensity'), skip_nans=True)
        points_array = np.fromiter(points_gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity',np.float32)])
        points_array = np.asarray(points_array)
        pc_as_numpy_ring = self.fix_data_type(points_array)
        lidar_points = pc_as_numpy_ring[:,:3]
        lidar_points = np.array(lidar_points,dtype=np.float32)
        intensities = pc_as_numpy_ring[:,3]  
        #lidar_points = self.trasnformation(lidar_points)[:,:3] 

        
        yaw_mask = self.filter_points_within_yaw(lidar_points, 60,60)
        lidar_points= lidar_points[yaw_mask]
        intensities = intensities[yaw_mask] 
        
        # Project the points
        
        projected_points, _ = cv2.projectPoints(lidar_points, self.rvec, self.tvec, self.matrix_coefficients, self.distortion_coefficients)
        # Convert points to integer for visualization
        projected_points = projected_points.squeeze().astype(int)
        


        # Image dimensions
        height, width = image.shape[:2]

        # Filter points within the image boundaries
        valid_points = []
        valid_intensities = []
        for i, point in enumerate(projected_points):
            x, y = point
            if 0 <= x < width and 0 <= y < height:
                valid_points.append((x, y))
                valid_intensities.append(intensities[i])
        

        # Convert to numpy array
        #valid_points = np.array(valid_points)
        valid_points = np.array(valid_points, dtype=np.float32).reshape(-1, 1, 2)
        valid_intensities = np.array(valid_intensities)
        # Undistort valid points
        undistorted_points = cv2.undistortPoints(valid_points, self.matrix_coefficients, self.distortion_coefficients, P=self.matrix_coefficients)
        undistorted_points = undistorted_points.reshape(-1, 2).astype(int)

        # Normalize intensity values to [0, 1] range
        normalized_intensities = (valid_intensities - valid_intensities.min()) / (valid_intensities.max() - valid_intensities.min())

        # Apply colormap
        colormap = plt.get_cmap('rainbow')
        colors = (colormap(normalized_intensities)[:, :3] * 255).astype(np.uint8)

        # Visualize valid LiDAR points on the image
        for point, color in zip(undistorted_points, colors):
            x, y = point
            cv2.circle(undistorted_image, (x, y), radius=1, color=(int(color[0]), int(color[1]), int(color[2])), thickness=2)
            #cv2.circle(image, (x, y), radius=3,color=[0,0,255] , thickness=-1)





        cv2.namedWindow("Image", cv2.WINDOW_NORMAL) 
    
        cv2.imshow('Image',undistorted_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()


def main(args=None):
    rclpy.init(args=args)
    publisher = camera_publisher()

    rclpy.spin( publisher  )       # execute simple_node 

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()