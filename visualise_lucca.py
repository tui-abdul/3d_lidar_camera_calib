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
        #with open('/home/abd1340m/Dokumente/ros2_overlay/src/overlay/overlay/0372_622_data/40253622.yaml', "r") as file_handle:
        with open('front_mid_teleop.yaml', "r") as file_handle:
        #with open('/home/abd1340m/Dokumente/os_0-webcam/valeo_cam.yaml', "r") as file_handle:
        #with open('calibrationdata_office_new_basler/ost.yaml', "r") as file_handle:     
        
            self.calib_data = yaml.safe_load(file_handle)
        
        matrix_coefficients =    self.calib_data["camera_matrix"]["data"]
        distortion_coefficients = self.calib_data["distortion_coefficients"]["data"]
        self.matrix_coefficients = np.array(matrix_coefficients).reshape(3,3)
        self.distortion_coefficients = np.array(distortion_coefficients)
    
        #image_color = '/basler_pole_a_left_id_103_sn_603/my_camera/pylon_ros2_camera_node/image_raw'  #pole a
        #ouster = '/ouster_pole_a_1108/points'  # pole a
        image_color = '/cam_teleoperation/front_mid'  
        ouster = '/points' 
                # Subscribe to topics
        image_sub = message_filters.Subscriber(self,Image,image_color)
        ouster = message_filters.Subscriber(self, PointCloud2,ouster,qos_profile= qos.qos_profile_sensor_data)#qos.ReliabilityPolicy.BEST_EFFORT)

        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub, ouster], queue_size=15, slop=0.05, allow_headerless=True)
        ats.registerCallback(self.callback)
        self.bridge = CvBridge()
        #self.rvec = np.array([ 0.7426531 , -1.55368898 , 1.73315511],dtype=np.float32).reshape((3,1)) #data from lab 0372_302
        #self.tvec = np.array([-0.12189352,-0.02106923,-0.04794924],dtype=np.float32).reshape((3,1)) #data from lab 0372_302
        #self.rvec = np.array([  0.76802442, -1.58597481 , 1.69252295],dtype=np.float32).reshape((3,1)) #data from lab 0372_302
        #self.tvec = np.array([-0.14695569,0.05743703,-0.03597695],dtype=np.float32).reshape((3,1)) #data from lab 0372_302
        #self.rvec,self.tvec = execute_code()
        #self.rvec = self.rvec#.reshape((3,1))
        #self.tvec = self.rvec#.reshape((3,1))
        #self.rvec = np.array([   0.88819031,-1.70450186, 1.63215907],dtype=np.float32).reshape((3,1)) # orignal lab data lidar 1108 and camera 618 (lab)
        #self.tvec = np.array([-0.15081952,  0.02194535, -0.04891262],dtype=np.float32).reshape((3,1)) # orignal lab data lidar 1108 and camera 618 (lab)

        #self.tvec = np.array([ 0.15307423, 0.01248773, -0.04994381],dtype=np.float64).reshape((3,1)) #data lidar 1108 and camera 603
        #self.rvec = np.array([ 1.48854854, -0.74412371,   0.67688705],dtype=np.float64).reshape((3,1)) #data lidar 1108 and camera 603

        #self.tvec = np.array([ -0.12755348, 0.03376902, 0.09379576],dtype=np.float64).reshape((3,1)) # intersection data lidar 1108 and camera 618
        #self.rvec = np.array([  0.82430635, -1.5835378 , 1.71078744],dtype=np.float64).reshape((3,1)) # intersection data lidar 1108 and camera 618

        #self.tvec = np.array([ -0.06127661, -0.09048259, 0.11473779],dtype=np.float64).reshape((3,1)) # lucca 1
        #self.rvec = np.array([   1.23048919, -1.21280759, 1.16656343],dtype=np.float64).reshape((3,1)) # lucca 1

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
        #cv2.imshow('Image',image)

        #while True:
        #key = cv2.waitKey(0)
        #if key == 27:  # 27 is the ASCII code for the Escape key
        #cv2.destroyAllWindows()
        #    exit()
        #pass

def main(args=None):
    rclpy.init(args=args)
    publisher = camera_publisher()

    rclpy.spin( publisher  )       # execute simple_node 

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()