import rclpy
import yaml
from sensor_msgs.msg import PointCloud2
from rclpy import  qos
from rclpy.node import Node
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import open3d as od3
import open3d as od31
#import ros2_numpy
from rclpy import  qos
import cv2.aruco as aruco
from aruco import my_estimatePoseSingleMarkers, getCornersInCameraWorld
import inspect
from bound_gui import create_slider_gui
#from sensor_msgs import point_cloud2
from sensor_msgs_py.point_cloud2 import read_points
from functools import partial
import tkinter as tk
from tkinter import ttk
import pyransac3d as pyrsc
#from pynput import keyboard
#from data.pyqt5_ import SliderGUI
from crop import *
from skspatial.objects import Line, Points, Plane
#from skspatial.plotting import plot_3d
import matplotlib.pyplot as plt 
import os
#from projection_test.projection import draw_pointcloud_color
#from data.open3d_slider_1 import *

#import open3d.visualization.gui as gui
#import open3d.visualization.rendering as rendering

class calib(Node):

    def __init__(self):
        super().__init__('publisher')
        with open('front_mid_teleop.yaml', "r") as file_handle:
            self.calib_data = yaml.safe_load(file_handle)

        matrix_coefficients =    self.calib_data["camera_matrix"]["data"]
        distortion_coefficients = self.calib_data["distortion_coefficients"]["data"]
        self.matrix_coefficients = np.array(matrix_coefficients).reshape(3,3)
        self.distortion_coefficients = np.array(distortion_coefficients)
        self.save_path = "/home/abd1340m/Dokumente/os_0-webcam/data/"
        image_color = '/cam_teleoperation/front_mid'
        ouster = '/points'
                # Subscribe to topics
        image_sub = message_filters.Subscriber(self,Image,image_color)
        ouster = message_filters.Subscriber(self, PointCloud2,ouster,qos_profile= qos.qos_profile_sensor_data)#qos.ReliabilityPolicy.BEST_EFFORT)

        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub, ouster], queue_size=20, slop=0.1, allow_headerless=True)
        ats.registerCallback(self.callbacks)
        #self.listener(image_color,ouster)
        
        self.pcd = od3.geometry.PointCloud()
        #self.vis.create_window()
        self.bridge = CvBridge()

        #initialize gui valies
        self.x_upper = 0  #"X Upper Bound",
        self.x_lower = 0  #"X Lower Bound",
        self.y_upper = 0  #"Y Upper Bound",
        self.y_lower = 0  #"Y Lower Bound",
        self.z_upper = 0  #"Z Upper Bound",
        self.z_lower = 0  #"Z Lower Bound"
        #self.app = gui.Application.instance
        #self.app.initialize()
        od3.utility.set_verbosity_level(od3.utility.VerbosityLevel.Debug)
        self.skip_flag = None
        self.points_2d = []
        self.points_3d = []  
        print('Initialization complete')
    def overlay(self,image_msg,lidar_msg,rvec,tvec):
        #image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        #image = cv2.cvtColor(image_msg, cv2.COLOR_GRAY2BGR)
        image = image_msg
        undistorted_image = cv2.undistort(image, self.matrix_coefficients,self.distortion_coefficients)
        points_gen =  read_points(lidar_msg, field_names=('x','y','z','intensity'), skip_nans=True)
        points_array = np.fromiter(points_gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity',np.float32)])
        points_array = np.asarray(points_array)
        pc_as_numpy_ring = self.fix_data_type(points_array)
        lidar_points = pc_as_numpy_ring[:,:3]
        lidar_points = np.array(lidar_points,dtype=np.float32)
        intensities = pc_as_numpy_ring[:,3]  
        lidar_points = self.trasnformation(lidar_points)[:,:3] 

        # Project the points
        
        projected_points, _ = cv2.projectPoints(lidar_points, rvec, tvec, self.matrix_coefficients, self.distortion_coefficients)
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
        colormap = plt.get_cmap('jet')
        colors = (colormap(normalized_intensities)[:, :3] * 255).astype(np.uint8)

        # Visualize valid LiDAR points on the image
        for point, color in zip(undistorted_points, colors):
            x, y = point
            cv2.circle(undistorted_image, (x, y), radius=3, color=(int(color[0]), int(color[1]), int(color[2])), thickness=-1)
            #cv2.circle(image, (x, y), radius=3,color=[0,0,255] , thickness=-1)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL) 
    
        cv2.imshow('Image',undistorted_image)
        #cv2.imshow('Image',image)

        
    
    def trasnformation(self,pc_as_numpy_array):
        t_mat =np.array([
                [-1, 0, 0, 0], 
                [0, -1, 0, 0], 
                [0, 0, 1, 0], 
                [0, 0, 0, 1]
            ])
        column_of_ones = np.ones((pc_as_numpy_array.shape[0] , 1))
        
        result_array = np.hstack((pc_as_numpy_array, column_of_ones))
        transformed_point_3d = np.dot(t_mat, result_array.T)
        #print('t_point',transformed_point_3d.shape)
        return transformed_point_3d.T
    
    def aruco_detect(self,frame):

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters()  # Marker detection parameters
        # lists of ids and the corners beloning to each id
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejectedCandidates = detector.detectMarkers(frame)     
        if corners:
            #rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, 0.393, self.matrix_coefficients, self.distortion_coefficients)   
            #dist = np.sqrt(tvec[0][0]**2 + tvec [0][1] **2 + tvec [0][2]**2)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            #cv2.putText(frame, f"Distance: {dist} m", (100, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            #marker_corners_3d = getCornersInCameraWorld(0.393,rvec=np.asarray(rvec),tvec=np.asarray(tvec))
            #print("3d marker corner",marker_corners_3d)
            pass
        else:
            print("corners not detected")
        return corners
    def marker_centroid(self,corners):
        print('marker centroid',corners.shape)
        centroid = np.mean(corners, axis=0)
        corners_plus_centroid = np.vstack((corners,centroid))    
        return corners_plus_centroid
    
    def pc_centroid(self,points):
        centroid = np.mean(points, axis=0)

        return centroid
    


    def project_points(self,img,points_2D):
        for k in range(len(points_2D)):
            x,y = points_2D[k][0][0].astype(int),points_2D[k][0][1].astype(int)
            if 0<x<len(img[0]) and 0<y<len(img[1]):
                cv2.circle(img, (x,y), 6,  (255, 0, 0), -1)
    def project_more_points(self,img,points_2D):
        for k in range(len(points_2D)):
            x,y = points_2D[k][0][0].astype(int),points_2D[k][0][1].astype(int)
            if 0<x<len(img[0]) and 0<y<len(img[1]):
                cv2.circle(img, (x,y), 3,  (0, 0, 255), -1)
    def project_more_more_points(self,img,points_2D):
        for k in range(len(points_2D)):
            x,y = points_2D[k][0][0].astype(int),points_2D[k][0][1].astype(int)
            #print('image',img.shape, img[0].shape,img[1].shape  )
            if 0<x<len(img[0]) and 0<y<len(img[1]):
                cv2.circle(img, (x,y), 1,  (0, 255, 0), -1)
        
    def re_proj_error(self,points2D_reproj,points2D ):
        
        points2D_reproj = points2D_reproj.reshape(points2D_reproj.shape[0] ,2)
   
        assert(points2D_reproj.shape == points2D.shape)
        error = (points2D_reproj - points2D)#[inliers]  # Compute error only over inliers.
        
        error = error.reshape(error.shape[0] ,2)
        rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
        
        return rmse
    
    def re_proj_error_comulative(self,points2D_reproj,points2D ):
        
        #points2D_reproj = points2D_reproj.reshape(5,2)
   
        if(points2D_reproj.shape == points2D.shape):
            error = (points2D_reproj - points2D)#[inliers]  # Compute error only over inliers.
        
            #error = error.reshape(5,2)
            rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
            print('comulative rmse',rmse)
        else:
            print('fix rmase, it is not working')
        
        return 0

    def calculate_ext_param(self,objectPoints,imagePoints,mat_intrinsic,dist_coeffs):
        print('object points at the end',objectPoints)
        print(' points at the end',imagePoints)
        assert(objectPoints.shape[0]  == imagePoints.shape[0] )
        success, rvec, tvec, inliers, = cv2.solvePnPRansac(objectPoints,imagePoints,mat_intrinsic,dist_coeffs,iterationsCount=100000, reprojectionError=0.5, confidence=0.99)
        '''
        # Solve PnP problem using solvePnPGeneric
        success, rvecs, tvecs, reprojectionErrors = cv2.solvePnPGeneric(
            objectPoints, imagePoints, mat_intrinsic, dist_coeffs,
            flags=cv2.SOLVEPNP_P3P
        )
        
        # Print results
        if success:
            #for i in range(len(rvecs)):
            #    print(f"Solution {i+1}:")
            print(f"Rotation Vector (rvec): \n{rvecs}")
            print(f"Translation Vector (tvec): \n{tvecs}")
            if reprojectionErrors is not None:
                print(f"Reprojection Error: {reprojectionErrors}")
        else:
            print("PnP solution not found.")
        '''
        return rvec, tvec
    def calculate_ext_param_comulative(self,objectPoints,imagePoints,mat_intrinsic,dist_coeffs):
        objectPoints =np.array(objectPoints)
        imagePoints = np.array(imagePoints)
        objectPoints = objectPoints.reshape(-1,objectPoints.shape[-1])
        imagePoints = imagePoints.reshape(-1,imagePoints.shape[-1])
        print('commulative ob points',objectPoints.shape)
        print('comulative image points',imagePoints.shape)
        assert(objectPoints.shape[0]  == imagePoints.shape[0] )
        success, rvec, tvec, inliers, = cv2.solvePnPRansac(objectPoints,imagePoints,mat_intrinsic,dist_coeffs,iterationsCount=1000000, reprojectionError=0.30, confidence=0.99)
        #rvec_refine, tvec_refine = cv2.solvePnPRefineLM(objectPoints[inliers],imagePoints[inliers], mat_intrinsic, dist_coeffs,rvec,tvec)
        return rvec, tvec
    
    def project_lidar_data(self,objectPoints, rvec, tvec, mat_intrinsic, dist_coeffs):
        print('object points data',objectPoints.shape)
        print('object points data',type(objectPoints))
        points_2D, _ = cv2.projectPoints(objectPoints,rvec,tvec,mat_intrinsic , dist_coeffs)
        return points_2D
    
    def select_obj_points(self,pc_as_numpy_array):
        obj_points = np.zeros((4,3))
        
        self.pcd.points = od3.utility.Vector3dVector(pc_as_numpy_array[:,:3] )
        # Visualize cloud and edit
        #self.vis = od3.visualization.VisualizerWithEditing()
        self.vis = od3.visualization.VisualizerWithVertexSelection()
        self.vis.create_window()
        self.vis.add_geometry(self.pcd)
        opt = self.vis.get_render_option()
        opt.show_coordinate_frame = True
        self.vis.run()
        pick_points_index = self.vis.get_picked_points() #[84, 119, 69]
        self.vis.destroy_window()
        #print(pc_as_numpy_array.shape)
        for i,index in enumerate(pick_points_index):
            obj_points[i]  = pc_as_numpy_array[index,:3] 
        return obj_points
    
    
    
    def select_roi_gui(self):
        update_slider = {} 
        def update_label(event):
            slider = event.widget
            name = slider_names[sliders.index(slider)]
            value = (slider.get() - 300) / 100
            label_values[name].config(text=f"{name}: {value:.2f}")
            update_slider[name] = value 

            


        def on_exit():
            root.destroy()

    # Create the main window
        root = tk.Tk()
        root.title("Slider GUI")
        root.geometry("600x300")

        # Create a frame to hold the sliders and labels
        frame = ttk.Frame(root, padding="20")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Slider names
        slider_names = [
            "X Upper Bound",
            "X Lower Bound",
            "Y Upper Bound",
            "Y Lower Bound",
            "Z Upper Bound",
            "Z Lower Bound"
        ]

        # Create six sliders
        sliders = []
        for i, name in enumerate(slider_names):
            slider = ttk.Scale(frame, from_=0, to=600, orient=tk.HORIZONTAL, length=400)
            slider.set(300)
            slider.grid(row=i, column=0, columnspan=2, sticky=tk.W+tk.E, padx=10, pady=5)
            slider.bind('<Motion>', update_label)
            sliders.append(slider)
        
        # Create labels to display the slider values
        label_values = {}
        for i, name in enumerate(slider_names):
            label = ttk.Label(frame, text=f"{name}: 0.00")
            label.grid(row=i, column=2, sticky=tk.W, padx=10)
            label_values[name] = label

        # initaial value
        #for i ,name in enumerate(slider_names):
        #    sliders[i].set(initial_value[i] )
        #    label_values[name].config(text=f"{name}: {initial_value[i] - 300/ 100 :.2f}")


        # Create an Exit button
        exit_button = ttk.Button(frame, text="Exit", command=on_exit)
        exit_button.grid(row=7, column=0, columnspan=3, pady=10)

        # Run the main loop
        root.mainloop()



        return update_slider

    '''
    def select_roi(self,pc_as_numpy_array):
        bounds = self.select_roi_gui()
        slider_value = bounds 
        print(slider_value)


        inrange = np.where( (pc_as_numpy_array[:, 2] > slider_value['Z Lower Bound'])  &
            (pc_as_numpy_array[:, 2] < slider_value['Z Upper Bound'])&
            (pc_as_numpy_array[:, 0] < slider_value['X Upper Bound'])   &
            (pc_as_numpy_array[:, 0] > slider_value['X Lower Bound'])  &
            (pc_as_numpy_array[:, 1] < slider_value['Y Upper Bound'])   &
            (pc_as_numpy_array[:, 1] > slider_value['Y Lower Bound']))

        pc_as_numpy_array = pc_as_numpy_array[inrange]        
        
        self.pcd.points = od3.utility.Vector3dVector(pc_as_numpy_array[:,:3])
        # Visualize cloud and edit
        #self.vis = od3.visualization.VisualizerWithEditing()
        self.vis = od3.visualization.VisualizerWithVertexSelection()
        self.vis.create_window()
        self.vis.add_geometry(self.pcd)
        opt = self.vis.get_render_option()
        opt.show_coordinate_frame = True
        self.vis.run()
        pick_points_index = self.vis.get_picked_points() #[84, 119, 69]
        #print(pick_points_index[0].coord  )
        self.vis.destroy_window()
        obj_points = np.zeros((len(pick_points_index),3))
        
        for i,point_obj in enumerate(pick_points_index):
            obj_points[i]  = pc_as_numpy_array[point_obj.index,:3] 

        return obj_points, pick_points_index
    '''
    def on_value_changed_x_lower(self,slider):
        self.x_lower = slider.double_value
        return self.x_lower
    
    def on_value_changed_x_upper(self,slider):
        self.x_upper = slider.double_value
        return self.x_upper
    
    def on_value_changed_y_lower(self,slider):
        self.y_lower = slider.double_value
        return self.y_lower
    def on_value_changed_y_upper(self,slider):
        self.y_upper = slider.double_value
        return self.y_upper
    def on_value_changed_z_lower(self,slider):
        self.z_lower = slider.double_value
        return self.z_lower
    def on_value_changed_z_upper(self,slider):
        self.z_upper = slider.double_value
        return self.z_upper

    def quit_callback(self,app):
        app.close()
        print('button')
    '''
    def create_window(self):

        self.window = self.app.create_window("Open3D", width = 400, height=250)
        print('hello')
        w = self.window
        
        em = w.theme.font_size
        
        margin = 0.5 * em
        
        layout = gui.Vert(0, gui.Margins(margin))
        self.window.add_child(layout)
        slider_x_lower = gui.Slider(gui.Slider.DOUBLE)
        slider_x_lower.set_limits(-3.0, +3.0)
        slider_x_lower.set_on_value_changed(lambda value: self.on_value_changed_x_lower(slider_x_lower))
        layout.add_child(slider_x_lower)

        slider_x_upper = gui.Slider(gui.Slider.DOUBLE)
        slider_x_upper.set_limits(-3.0, +3.0)
        slider_x_upper.set_on_value_changed(lambda value: self.on_value_changed_x_upper(slider_x_upper))
        layout.add_child(slider_x_upper)

        slider_y_lower = gui.Slider(gui.Slider.DOUBLE)
        slider_y_lower.set_limits(-3.0, +3.0)
        slider_y_lower.set_on_value_changed(lambda value: self.on_value_changed_y_lower(slider_y_lower))
        layout.add_child(slider_y_lower)

        slider_y_upper = gui.Slider(gui.Slider.DOUBLE)
        slider_y_upper.set_limits(-3.0, +3.0)
        slider_y_upper.set_on_value_changed(lambda value: self.on_value_changed_y_upper(slider_y_upper))
        layout.add_child(slider_y_upper)

        slider_z_lower = gui.Slider(gui.Slider.DOUBLE)
        slider_z_lower.set_limits(-3.0, +3.0)
        slider_z_lower.set_on_value_changed(lambda value: self.on_value_changed_z_lower(slider_z_lower))
        layout.add_child(slider_z_lower)

        slider_z_upper = gui.Slider(gui.Slider.DOUBLE)
        slider_z_upper.set_limits(-3.0, +3.0)
        slider_z_upper.set_on_value_changed(lambda value: self.on_value_changed_z_upper(slider_z_upper))
        layout.add_child(slider_z_upper)
        
        
        quit_button = gui.Button("Quit")
        quit_button.set_on_clicked(lambda app: self.quit_callback(self.window))
        layout.add_child(quit_button) 
        
        return  self.window
    
    def main_slider(self):

        self.create_window()
        
        self.app.run()
        #self.window.close()
        #self.app.quit()
        
    
        #del self.window
    ''' 

    def select_roi(self,pc_as_numpy_array):
    
        

        '''
        inrange = np.where( (pc_as_numpy_array[:, 2] > self.z_lower )  &
            (pc_as_numpy_array[:, 2] < self.z_upper )&
            (pc_as_numpy_array[:, 0] < self.x_upper )   &
       (pc_as_numpy_array[:, 0] >  self.x_lower )  &
            (pc_as_numpy_array[:, 1] < self.y_upper )   &
            (pc_as_numpy_array[:, 1] > self.y_lower ))

            pc_as_numpy_array = pc_as_numpy_array[inrange]       
            ''' 
        while True:
            print("Looping...")
            # Your code here...
            
            

            new, ind = edit(pc_as_numpy_array[:,:3] )
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(new)
            vis = o3d.visualization.Visualizer()
            vis.create_window('Croped View')
            vis.add_geometry(pcd)
            vis.run()
            vis.destroy_window()
            
            user_input = input("Press 'm' to break, 'c' to continue, or 'ESC' to exit: or 'skip' ")
            if user_input == 'm':
                print("Breaking the loop...")
                break
            elif user_input == 'c':
                print("Continuing the loop...")
                continue
            elif user_input.lower() == 'esc':
                print("Exiting the program...")
                break
            elif user_input == 'skip':
                self.skip_flag = 'skip'
                break
            else:
                print("Invalid input. Please try again.")
        if self.skip_flag == 'skip':
            return
        pc_as_numpy_array = pc_as_numpy_array[ind] 
        self.pcd.points = od3.utility.Vector3dVector(pc_as_numpy_array[:,:3])
        # Visualize cloud and edit
        #self.vis = od3.visualization.VisualizerWithEditing()
        self.vis = od3.visualization.VisualizerWithVertexSelection()
        #self.vis = od3.visualization.Visualizer()
        self.vis.create_window('Window for Points Selection')
        self.vis.add_geometry(self.pcd)
        opt = self.vis.get_render_option()
        opt.show_coordinate_frame = True

        self.vis.run()
        
        pick_points_index = self.vis.get_picked_points() #[84, 119, 69]
        
        self.vis.destroy_window()
        obj_points = np.zeros((len(pick_points_index),4))
        #print(pick_points_index  )
        
        for i,point_obj in enumerate(pick_points_index):
            obj_points[i]  = pc_as_numpy_array[point_obj.index,:] 
        
        return obj_points
    
    
    def visualize(self,points):
        points_vis = od3.geometry.PointCloud()
        points_vis.points = od3.utility.Vector3dVector(points)
        self.vis = od3.visualization.VisualizerWithVertexSelection()
        self.vis.create_window()
        self.vis.add_geometry(points_vis)
        opt = self.vis.get_render_option()
        opt.show_coordinate_frame = True
        self.vis.run()
        #pick_points_index = self.vis.get_picked_points() #[84, 119, 69]
        #print(pick_points_index[0].coord  )
        self.vis.destroy_window()
        
    def corner_refine(self,img, aruco_corners):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        #aruco_corners = np.array(aruco_corners, dtype=np.float32)
        
        refined_corners = cv2.cornerSubPix(img, aruco_corners, (5, 5), (-1, -1), criteria)
        return refined_corners
    def undistort(self,frame,matrix_coefficients,distortion_coefficients):
        undist_img = cv2.undistort(frame,matrix_coefficients,distortion_coefficients)
        return undist_img
    def pcd_conversion(self,array_numpy):
        pcd = od31.geometry.PointCloud()

        # Set the point cloud data from the NumPy array
        pcd.points = od31.utility.Vector3dVector(np.asarray(array_numpy))

        # Save the point cloud to a PCD file
        od31.io.write_point_cloud(self.save_path + "point_cloud_full_1.pcd", pcd)

    def plane_equation(self,roi):
   
        projected_cloud = od3.geometry.PointCloud()
        projected_cloud.points = od3.utility.Vector3dVector(roi)
        print('waiting for calculating plane equation calculation')
        plane_model, inliers = projected_cloud.segment_plane(distance_threshold=0.005,
                                                ransac_n=12,
                                                num_iterations=100000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        print('plane equation inliers',inliers)
        #print('pcd',pcd)
        #print('inliers',inliers,len(inliers))
        plane = [a, b, c, d]
        return plane
    

    def line_detect(self,points, all_points):
        line = pyrsc.Line()
        print("points",points.shape)
        # Fit line to the data
        A, B, inliers = line.fit(points, thresh=0.2, maxIteration=1000)

        print(f"A (3D slope of the line): {A}")
        print(f"B (Axis interception): {B}")

        # Convert inliers to Open3D point cloud
        inlier_points = points[inliers]
        point_cloud = od3.geometry.PointCloud()
        point_cloud.points = od3.utility.Vector3dVector(all_points)

        # Create line segment for visualization
        line_segment = od3.geometry.LineSet()
        line_points = np.vstack([B, B + A])
        line_segment.points = od3.utility.Vector3dVector(line_points)
        line_segment.lines = od3.utility.Vector2iVector([[0, 1]])

        # Create visualizer
        visualizer = od3.visualization.Visualizer()
        visualizer.create_window()

        # Add point cloud and line segment to visualizer
        visualizer.add_geometry(point_cloud)
        visualizer.add_geometry(line_segment)

        # Run visualizer
        visualizer.run()

        # Close visualizer
        visualizer.destroy_window()
    def line_detect_scipy(self,points):
        points = Points(points)
        line_fit = Line.best_fit(points)
        print(line_fit)
        '''
        plot_3d(
        line_fit.plotter(t_1=-7, t_2=7, c='k'),
        points.plotter(c='b', depthshade=False),
        )
        #plt.show()
        '''
        return line_fit
    def line_intersection(self,line_a,line_b):
        point = line_a.intersect_line(line_b, check_coplanar = False)
        print('intersection',point)
        return point

    def select_line(self,boundary_points):

        vis = o3d.visualization.VisualizerWithVertexSelection()
        vis.create_window('select line')
        boundary_pcd = o3d.geometry.PointCloud()
        boundary_pcd.points = o3d.utility.Vector3dVector(boundary_points)
        vis.add_geometry( boundary_pcd)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()
        pick_points_index = vis.get_picked_points()
        obj_points = np.zeros((len(pick_points_index),3))

        for i,point_obj in enumerate(pick_points_index):
            obj_points[i]  = boundary_points[point_obj.index,:] 
        print('line selection points',obj_points)
        return obj_points
    
    def plot_rectangle(self,plane, corners_3d):
        lines = [[0, 1], [1, 2], [2, 3], [3, 0]]

        # Create LineSet object
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(corners_3d)
        line_set.lines = o3d.utility.Vector2iVector(lines)

        # Create PointCloud object for points
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(corners_3d)

        # Color the points for highlighting
        point_colors = np.array([[1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0]])  # Red color for points
        point_cloud.colors = o3d.utility.Vector3dVector(point_colors)
        plane_cloud = o3d.geometry.PointCloud()
        plane_cloud.points = o3d.utility.Vector3dVector(np.asarray(plane))
        # Create visualizer object
        vis = o3d.visualization.Visualizer()

        # Add geometries to the visualizer
        vis.create_window()
        vis.add_geometry(line_set)
        vis.add_geometry(point_cloud)
        vis.add_geometry(plane_cloud)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        # Run the visualizer
        vis.run()
        vis.destroy_window()
    def find_point_on_plane(self,a, b, c, d):
        """
        Find a point on the plane ax + by + cz + d = 0 by setting x and y to 0 and solving for z.

        Args:
        a (float): Coefficient of x in the plane equation.
        b (float): Coefficient of y in the plane equation.
        c (float): Coefficient of z in the plane equation.
        d (float): Constant term in the plane equation.

        Returns:
        tuple: A point (x, y, z) on the plane.
        """
        # Set x = 0 and y = 0
        x = 0
        y = 0
        
        # Solve for z
        if c != 0:
            z = -d / c
            return (x, y, z)
        else:
            raise ValueError("Coefficient c cannot be zero, as it would result in division by zero.")
    def plane_fitting(self,plane_model,object_points):


        a,b,c,d = plane_model
        point = self.find_point_on_plane(a, b, c, d)
        plane = Plane(point= point, normal=[a,b,c] )
        points_pro = Points(object_points[:,:3] )
        point_projected =[]# np.zeros(object_points.shape)
        for x in points_pro:
            out = plane.project_point(x)
            point_projected.append(out)
        point_projected = np.array(point_projected)
        projected_points = np.hstack((np.asarray(point_projected),object_points[:,3].reshape(-1, 1)) )
        #print('projected_points', projected_points)
        test = projected_points[(projected_points[:, 3] == np.min(projected_points[:, 3])) | (projected_points[:, 3] == np.max(projected_points[:, 3]))] 
        #print('test',test)
        projected_cloud = od3.geometry.PointCloud()
        projected_cloud.points = od3.utility.Vector3dVector(projected_points[:,:3] )
        #projected_cloud.points = od3.utility.Vector3dVector(np.asarray(board_points))
        # Plot the projected points
        #o3d.visualization.draw_geometries([projected_cloud])
        vis = od3.visualization.VisualizerWithEditing()
        vis.create_window('plane_fitting')
        vis.add_geometry(projected_cloud)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()
        pick_points_index = vis.get_picked_points()
        print('picked_points',pick_points_index)
        points = np.asarray(projected_cloud.points)[pick_points_index] 
        print(points)
        #return np.asarray(projected_cloud.points)
        print('plane fitting',projected_points)
        return points, projected_points[:,:3], projected_points
    
    def sort_scan_beam(self, points):
        #top_edge = points[points[:, 3] == np.min(points[:, 3])]
        #bottom_edge = points[points[:, 3] == np.max(points[:, 3])]
        sorted_point_cloud = points#[np.argsort(points[:, 3])]

        # Print the rearranged point cloud
        print('sorted point cloud',sorted_point_cloud)
        return sorted_point_cloud
    
    def fix_scan_beams(self,scan_beams):
        new_beams = np.empty((0, 4), dtype=float)
        values, indexes = np.unique(scan_beams[:,3] ,return_index=True)

        for scan in values:
            index = np.where(scan_beams[:,3] == scan)[0]
            #print('index',index)
            if len(index) > 1:
                line_fit = Line.best_fit(scan_beams[index,:3]  ) 
                for pont in (scan_beams[index,:3]):
                    each_point = line_fit.project_point(pont)
                    each_point = np.append(each_point,scan)
                    new_beams = np.vstack((new_beams,each_point))
            else:
                new_beams = np.vstack((new_beams,scan_beams[index,:]))
        #print('input_beam',scan_beams)
        #print('new_beam',new_beams)
        return new_beams
    
    def find_edges(self,sorted_point_cloud):
        bound = [] 
        top_edge = sorted_point_cloud[sorted_point_cloud[:, 3] == np.min(sorted_point_cloud[:, 3])]
        bottom_edge = sorted_point_cloud[sorted_point_cloud[:, 3] == np.max(sorted_point_cloud[:, 3])]

        min_beam = np.min(sorted_point_cloud[:,3])
        max_beam = np.max(sorted_point_cloud[:,3])
        left_edge = []
        right_edge = []  
        for i in range(int(max_beam-min_beam) + 1):
            beam_array = sorted_point_cloud[sorted_point_cloud[:,3] == min_beam + i]
            left_edge.append(beam_array[np.where(beam_array[:,1]  == np.max(beam_array[:,1]))]) 
            right_edge.append(beam_array[np.where(beam_array[:,1]  == np.min(beam_array[:,1]))])             
        
        left_edge = np.squeeze(np.array(left_edge))[:,:3] 
        right_edge = np.squeeze(np.array(right_edge))[:,:3] 
        #top_edge = np.squeeze(top_edge)[:,:3] 
        #bottom_edge = np.squeeze(bottom_edge)[:,:3] 

        bound.append(left_edge)
        bound.append(right_edge)
        bound = np.squeeze(np.asarray(bound))
        bound = bound.reshape(bound.shape[0]*bound.shape[1],3 )
        #print('bound',bound.shape)
        #visualize
        
        #print('sorted_pc',sorted_point_cloud)
        projected_cloud = od3.geometry.PointCloud()
        projected_cloud.points = od3.utility.Vector3dVector(sorted_point_cloud[:,:3]   )
        vis = od3.visualization.VisualizerWithEditing()
        vis.create_window('edges')
        vis.add_geometry(projected_cloud)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()
        '''
        projected_cloud = od3.geometry.PointCloud()
        projected_cloud.points = od3.utility.Vector3dVector(sorted_point_cloud[:,:3] )

        #boundarys, mask = projected_cloud.compute_boundary_points(0.02, 30)
        # TODO: not good to get size of points.
        #print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {projected_cloud.point.positions.shape[0]} points.")

        boundarys = boundarys.paint_uniform_color([1.0, 0.0, 0.0])
        projected_cloud = projected_cloud.paint_uniform_color([0.6, 0.6, 0.6])
        od3.visualization.draw_geometries([projected_cloud.to_legacy(), boundarys.to_legacy()],
                                              zoom=0.3412,
                                              front=[0.3257, -0.2125, -0.8795],
                                              lookat=[2.6172, 2.0475, 1.532],
                                              up=[-0.0694, -0.9768, 0.2024])
        '''
        return bound#left_edge, right_edge, top_edge, bottom_edge

    def transformation_matrix(self,rvec, tvec):
        R_lidar_to_camera, _ = cv2.Rodrigues(rvec)

        # Compose transformation matrix T (LIDAR to camera)
        T_lidar_to_camera = np.eye(4)
        T_lidar_to_camera[:3, :3] = R_lidar_to_camera
        
        T_lidar_to_camera[:3, 3] = tvec.flatten()
        #print('T_lidar_to_camera',T_lidar_to_camera)
        # Invert T to get camera to LIDAR transformation
        T_camera_to_lidar = np.linalg.inv(T_lidar_to_camera)

        # Projection matrix from camera to LIDAR
        # Projection matrix from camera to LIDAR
        #P_camera_to_lidar = np.vstack((T_camera_to_lidar[:3], [0, 0, 0, 1]))

        print("Projection Matrix (LIDAR to Camera):\n", T_lidar_to_camera)
        print("Projection Matrix (Camera to LIDAR):\n", T_camera_to_lidar)
        print("T distance", np.sqrt(tvec[0]**2 +tvec[1]**2 + tvec[2]**2) )

        #return T_lidar_to_camera, T_camera_to_lidar
    def fix_data_type(self, array):
        points_array_ring = np.zeros((65536, 4))
        for i, point in enumerate(array):
            x,y,z, ring = point
            points_array_ring[i,0] = x
            points_array_ring[i,1] = y
            points_array_ring[i,2] = z
            points_array_ring[i,3] = ring 
        #print(points_array_ring)
        return points_array_ring  

    def callbacks(self,image, ouster):  
        print('new frame')
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = self.aruco_detect(frame)
        if not corners:
            return
        else:
            cv2.namedWindow("Image", cv2.WINDOW_NORMAL) 
            #cv2.imshow('Image',undistort_frame)
            cv2.imshow('Image',frame)
            while True:
                key = cv2.waitKey(0)
                if key == 27:  # 27 is the ASCII code for the Escape key
                    cv2.destroyAllWindows()
                    break
        #points_array_ring = np.array((65536,4))
        points_gen =  read_points(ouster, field_names=('x','y','z','ring'), skip_nans=True)
        points_array = np.fromiter(points_gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('ring',np.float32)])
        points_array = np.asarray(points_array)
        pc_as_numpy_ring = self.fix_data_type(points_array)
        #print("start again",pc_as_numpy_ring)
        #pc_as_numpy_array = np.array(ros2_numpy.point_cloud2.point_cloud2_to_array(ouster)['xyz'])
        #print('resolution2',pc_as_numpy_array.shape)
        #print(np.array_equal(pc_as_numpy_array[250:300,:] ,pc_as_numpy_ring[250:300,:] ))
        #pc_as_numpy_array = self.trasnformation(pc_as_numpy_ring[:,:3] )
        #print("shape of lidar points",pc_as_numpy_array[:,:3].shape)
        
        #print("shape of lidar points",pc_as_numpy_array.shape)
        #pc_as_numpy_array = np.hstack((pc_as_numpy_array[:,:3],pc_as_numpy_ring[:,3].reshape(-1,1)))
        #roi = self.select_roi(pc_as_numpy_array)
       
        roi = self.select_roi(pc_as_numpy_ring)
        print('roi',roi)
        print('in the main callback')
        if self.skip_flag == 'skip':
            self.skip_flag = None
            return
        #print(roi.shape)
        #self.select_roi_gui()
        #print('roi', roi)
        #if roi.size == 0:
        #    print("get next frame ")
        #    pass
        
        plane_equation = self.plane_equation(roi[:,:3] )
        obj_points, rest_of_points, plane_fit_with_ring_points = self.plane_fitting(plane_equation,roi)
        
        
        sorted_beams = self.sort_scan_beam(plane_fit_with_ring_points)
        new_beams = self.fix_scan_beams(sorted_beams)
        #self.pcd_conversion(new_beams[:,:3]   )
        bound = self.find_edges(new_beams)
        points1 = self.select_line(bound)
        line1 = self.line_detect_scipy(points1)
        points2 = self.select_line(bound)
        line2 = self.line_detect_scipy(points2)
        points3 = self.select_line(bound)
        line3 = self.line_detect_scipy(points3)
        points4 = self.select_line(bound)
        line4 = self.line_detect_scipy(points4)
        corner_3d_1 = self.line_intersection(line1,line2)
        corner_3d_2 = self.line_intersection(line2,line3)
        corner_3d_3 = self.line_intersection(line3,line4)
        corner_3d_4 = self.line_intersection(line4,line1)
        #self.line_detect(left  ,rest_of_points)
        #self.line_detect(right,rest_of_points)
        #self.line_detect(top,rest_of_points)
        #self.line_detect(bottom,rest_of_points)
        #self.visualize(rest_of_points[:20,:])
        #self.pcd_conversion(roi)
        #exit()
        #obj_points = self.select_obj_points(pc_as_numpy_array)
        obj_points = np.vstack((corner_3d_1,corner_3d_2,corner_3d_3,corner_3d_4))
        
        print('obj_points shape',obj_points.shape)
        self.plot_rectangle(new_beams[:,:3] ,obj_points)
        corners = np.squeeze(corners[0])
        corners = self.corner_refine(grey,corners)
        print('corner',corners)
        corners = self.marker_centroid(corners)
        pc_centroid = self.pc_centroid(new_beams[:,:3])
        print('obj points', obj_points)
        obj_points = np.vstack((obj_points,pc_centroid))
        undistort_frame = self.undistort(frame,self.matrix_coefficients,self.distortion_coefficients)
        rvec ,tvec = self.calculate_ext_param(obj_points[:4,:] ,corners[:4,:] ,self.matrix_coefficients,self.distortion_coefficients)
        
        self.transformation_matrix(rvec, tvec)
        
        points2D_reproj = self.project_lidar_data(obj_points, rvec, tvec, self.matrix_coefficients, self.distortion_coefficients)
        #points2D_more_points = self.project_lidar_data(rest_of_points, rvec, tvec, self.matrix_coefficients, self.distortion_coefficients)
        #roi1 = self.select_roi(pc_as_numpy_array)
        #print("roi1",roi1)
        #points2D_more_more_points = self.project_lidar_data(roi1, rvec, tvec, self.matrix_coefficients, self.distortion_coefficients)
        print('proj points',points2D_reproj)
        #self.project_points(undistort_frame,points2D_reproj)
        self.project_points(frame,points2D_reproj)
        #   self.project_more_points(undistort_frame,points2D_more_points)
        #self.project_more_more_points(undistort_frame,points2D_more_more_points)
        rmse = self.re_proj_error(points2D_reproj,corners )
        print('rmse',rmse)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL) 
        cv2.imshow('Image',frame)
        #cv2.imshow('Image',undistort_frame)

        #draw_pointcloud_color(pc_as_numpy_ring[:,:3] , undistort_frame, rvec, tvec, self.matrix_coefficients, [1200,800] )
        if rmse < 20:
            #self.points_2d.append(corners)
            #self.points_3d.append(obj_points)
            #object_pts = np.array(self.points_3d)
            #object_pts = object_pts.reshape(object_pts.shape[0]*object_pts.shape[1],object_pts.shape[2])
            #rvec_add ,tvec_add = self.calculate_ext_param_comulative(object_pts,self.points_2d,self.matrix_coefficients,self.distortion_coefficients)
            #points2D_reproj = self.project_lidar_data(object_pts, rvec_add, tvec_add, self.matrix_coefficients, self.distortion_coefficients)
            #rmse = self.re_proj_error_comulative(np.squeeze(points2D_reproj),np.squeeze(np.asarray(self.points_2d)) )
            #print('commulative rmse',rmse)
            #self.transformation_matrix(rvec_add ,tvec_add)
            filename_1 = 'img_points.txt'
            filename_2 = 'lidar_points.txt'
            if os.path.exists(filename_1 and filename_2):
                # Append the additional array to the text file
                with open('img_points.txt', 'a') as f:
                    np.savetxt(f, corners, delimiter=',', fmt='%.2f')
                with open('lidar_points.txt', 'a') as f:
                    np.savetxt(f, obj_points, delimiter=',', fmt='%.2f')
            else:
                # Save the initial array to a text file
                np.savetxt('img_points.txt', corners, delimiter=',', fmt='%.2f')
                np.savetxt('lidar_points.txt', obj_points, delimiter=',', fmt='%.2f')
            #self.overlay(frame,ouster,rvec_add,tvec_add)
            if os.path.exists(filename_1 and filename_2):
                with open(filename_1, 'r') as file1:
                    line_count1 = sum(1 for line in file1)
                    print('image file line count',line_count1)
                with open(filename_1, 'r') as file2:
                    line_count2 = sum(1 for line in file2)
                    print('image file line count',line_count2)

        while True:
            key = cv2.waitKey(0)
            if key == 27:  # 27 is the ASCII code for the Escape key
                cv2.destroyAllWindows()
                break
                
        self.skip_flag = None
        

def main(args=None):
    rclpy.init(args=args)
    publisher = calib()

    rclpy.spin( publisher  )       # execute simple_node 

    publisher.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
    # Start subscriber
    