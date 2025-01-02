import numpy as np
import cv2
import yaml
from utility_functions.read_txt import format_lidar_data


with open("param.yaml","r") as file_handler:
        load_data = yaml.safe_load(file_handler)

with open(load_data["camera_intrinsic_param"], "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
        matrix_coefficients =    calib_data["camera_matrix"]["data"]
        distortion_coefficients = calib_data["distortion_coefficients"]["data"]
        matrix_coefficients = np.array(matrix_coefficients).reshape(3,3)
        distortion_coefficients = np.array(distortion_coefficients)





def calculate_ext_param_comulative(objectPoints,imagePoints,mat_intrinsic,dist_coeffs):
    objectPoints =np.array(objectPoints)
    imagePoints = np.array(imagePoints)

    objectPoints = objectPoints.reshape(-1,objectPoints.shape[-1])
    imagePoints = imagePoints.reshape(-1,imagePoints.shape[-1])

    assert(objectPoints.shape[0]  == imagePoints.shape[0] )
    success, rvec, tvec, inliers, = cv2.solvePnPRansac(objectPoints,imagePoints,mat_intrinsic,dist_coeffs,iterationsCount=1000000,reprojectionError=1.5,confidence=0.99)

    print('tvec',tvec)
    print('rvec',rvec)
    print('inliers',inliers)
    points2D_reproj = project_lidar_data(objectPoints, rvec, tvec, mat_intrinsic, dist_coeffs)
    points2D_reproj = np.squeeze(points2D_reproj)
    print('sucess', success)

    
    if(points2D_reproj.shape == imagePoints.shape):
        error = (points2D_reproj - imagePoints)[inliers]  # Compute error only over inliers.
        error = np.squeeze(error)
        rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
        print('comulative rmse',rmse)
    return rvec, tvec

def project_lidar_data(objectPoints, rvec, tvec, mat_intrinsic, dist_coeffs):

    points_2D, _ = cv2.projectPoints(objectPoints,rvec,tvec,mat_intrinsic , dist_coeffs)
    return points_2D


def transformation_matrix(rvec, tvec):
    R_lidar_to_camera, _ = cv2.Rodrigues(rvec)

    # Compose transformation matrix T (LIDAR to camera)
    T_lidar_to_camera = np.eye(4)
    T_lidar_to_camera[:3, :3] = R_lidar_to_camera
    
    T_lidar_to_camera[:3, 3] = tvec.flatten()

    T_camera_to_lidar = np.linalg.inv(T_lidar_to_camera)



    print("Projection Matrix (LIDAR to Camera):\n", T_lidar_to_camera)
    print("Projection Matrix (Camera to LIDAR):\n", T_camera_to_lidar)
    print("T distance", np.sqrt(tvec[0]**2 +tvec[1]**2 + tvec[2]**2) )








img = np.loadtxt('img_points.txt', delimiter=',')#[:10,:] 
obj = np.loadtxt('lidar_points.txt', delimiter=',')#[:10,:]   





rvec ,tvec = calculate_ext_param_comulative(obj[:,:]     ,img[:,:]     ,matrix_coefficients,distortion_coefficients)
transformation_matrix(rvec, tvec)




