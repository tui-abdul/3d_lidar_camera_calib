import rclpy
import yaml
from sensor_msgs.msg import PointCloud2
from rclpy import  qos
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import open3d as od3
from rclpy import  qos
import cv2.aruco as aruco
from sensor_msgs_py.point_cloud2 import read_points
from utility_functions.crop import *
from skspatial.objects import Line, Points, Plane
import os


class calib(Node):

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
        #self.save_path = "/home/abd1340m/Dokumente/os_0-webcam/data/"
        image_color = load_data["camera_topic"] 
        ouster = load_data["lidar_topic"] 
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


        od3.utility.set_verbosity_level(od3.utility.VerbosityLevel.Debug)
        self.skip_flag = None
        self.points_2d = []
        self.points_3d = []  
        print('Initialization complete')

        
    ####### irrelevent #####
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
            
            cv2.aruco.drawDetectedMarkers(frame, corners)
            
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
    
        
    def re_proj_error(self,points2D_reproj,points2D ):
        
        points2D_reproj = points2D_reproj.reshape(points2D_reproj.shape[0] ,2)
   
        assert(points2D_reproj.shape == points2D.shape)
        error = (points2D_reproj - points2D)#[inliers]  # Compute error only over inliers.
        
        error = error.reshape(error.shape[0] ,2)
        rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
        
        return rmse


    def calculate_ext_param(self,objectPoints,imagePoints,mat_intrinsic,dist_coeffs):
        print('object points at the end',objectPoints)
        print(' points at the end',imagePoints)
        assert(objectPoints.shape[0]  == imagePoints.shape[0] )
        success, rvec, tvec, inliers, = cv2.solvePnPRansac(objectPoints,imagePoints,mat_intrinsic,dist_coeffs,iterationsCount=100000, reprojectionError=0.5, confidence=0.99)
        
        return rvec, tvec
    
    
    def project_lidar_data(self,objectPoints, rvec, tvec, mat_intrinsic, dist_coeffs):
        print('object points data',objectPoints.shape)
        print('object points data',type(objectPoints))
        points_2D, _ = cv2.projectPoints(objectPoints,rvec,tvec,mat_intrinsic , dist_coeffs)
        return points_2D
    

    

    def select_roi(self,pc_as_numpy_array):

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

        self.vis = od3.visualization.VisualizerWithVertexSelection()
    
        self.vis.create_window('Window for Points Selection')
        self.vis.add_geometry(self.pcd)
        opt = self.vis.get_render_option()
        opt.show_coordinate_frame = True

        self.vis.run()
        
        pick_points_index = self.vis.get_picked_points() #[84, 119, 69]
        
        self.vis.destroy_window()
        obj_points = np.zeros((len(pick_points_index),4))
    
        
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
     
        plane = [a, b, c, d]
        return plane
    


    def line_detect_scipy(self,points):
        points = Points(points)
        line_fit = Line.best_fit(points)
        print(line_fit)
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
        
        test = projected_points[(projected_points[:, 3] == np.min(projected_points[:, 3])) | (projected_points[:, 3] == np.max(projected_points[:, 3]))] 
        
        projected_cloud = od3.geometry.PointCloud()
        projected_cloud.points = od3.utility.Vector3dVector(projected_points[:,:3] )
        
        vis = od3.visualization.VisualizerWithEditing()
        vis.create_window('plane_fitting')
        vis.add_geometry(projected_cloud)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()
        pick_points_index = vis.get_picked_points()
        
        points = np.asarray(projected_cloud.points)[pick_points_index] 
        
       
        return points, projected_points[:,:3], projected_points
    
    def sort_scan_beam(self, points):        
        sorted_point_cloud = points
        return sorted_point_cloud
    
    def fix_scan_beams(self,scan_beams):
        new_beams = np.empty((0, 4), dtype=float)
        values, indexes = np.unique(scan_beams[:,3] ,return_index=True)
        for scan in values:
            index = np.where(scan_beams[:,3] == scan)[0]
            
            if len(index) > 1:
                line_fit = Line.best_fit(scan_beams[index,:3]  ) 
                for pont in (scan_beams[index,:3]):
                    each_point = line_fit.project_point(pont)
                    each_point = np.append(each_point,scan)
                    new_beams = np.vstack((new_beams,each_point))
            else:
                new_beams = np.vstack((new_beams,scan_beams[index,:]))
        
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
       

        bound.append(left_edge)
        bound.append(right_edge)
        bound = np.squeeze(np.asarray(bound))
        bound = bound.reshape(bound.shape[0]*bound.shape[1],3 )
       
        
        
        projected_cloud = od3.geometry.PointCloud()
        projected_cloud.points = od3.utility.Vector3dVector(sorted_point_cloud[:,:3]   )
        vis = od3.visualization.VisualizerWithEditing()
        vis.create_window('edges')
        vis.add_geometry(projected_cloud)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()
        
        return bound#left_edge, right_edge, top_edge, bottom_edge

    def transformation_matrix(self,rvec, tvec):
        R_lidar_to_camera, _ = cv2.Rodrigues(rvec)

        # Compose transformation matrix T (LIDAR to camera)
        T_lidar_to_camera = np.eye(4)
        T_lidar_to_camera[:3, :3] = R_lidar_to_camera
        
        T_lidar_to_camera[:3, 3] = tvec.flatten()
        
        T_camera_to_lidar = np.linalg.inv(T_lidar_to_camera)

       

        print("Projection Matrix (LIDAR to Camera):\n", T_lidar_to_camera)
        print("Projection Matrix (Camera to LIDAR):\n", T_camera_to_lidar)
        print("T distance", np.sqrt(tvec[0]**2 +tvec[1]**2 + tvec[2]**2) )

        
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
        
        points_gen =  read_points(ouster, field_names=('x','y','z','ring'), skip_nans=True)
        points_array = np.fromiter(points_gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('ring',np.float32)])
        points_array = np.asarray(points_array)
        pc_as_numpy_ring = self.fix_data_type(points_array)
        
       
        roi = self.select_roi(pc_as_numpy_ring)
        print('roi',roi)
        print('in the main callback')
        if self.skip_flag == 'skip':
            self.skip_flag = None
            return
        
        
        plane_equation = self.plane_equation(roi[:,:3] )
        obj_points, rest_of_points, plane_fit_with_ring_points = self.plane_fitting(plane_equation,roi)
        
        
        sorted_beams = self.sort_scan_beam(plane_fit_with_ring_points)
        new_beams = self.fix_scan_beams(sorted_beams)
        
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
        
        print('proj points',points2D_reproj)
        
        self.project_points(frame,points2D_reproj)
        
        rmse = self.re_proj_error(points2D_reproj,corners )
        print('rmse',rmse)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL) 
        cv2.imshow('Image',frame)
       
        if rmse < 20:
            
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
    