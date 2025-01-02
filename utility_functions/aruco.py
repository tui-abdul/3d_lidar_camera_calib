import cv2
import cv2.aruco as aruco
import numpy as np
import yaml


def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    #print(corners)
    for c in corners:
        #print(marker_points.shape,c.shape)
        #print(cMat.shape,dcoeff.shape)
        nada, R, t = cv2.solvePnP(marker_points, c, np.asarray(mtx), np.asarray(distortion) , False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


def getCornersInCameraWorld(marker_size, rvec, tvec):
    


    # Convert the rotation vector to a rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    
    # Invert the rotation matrix
    R_inv = R.T
    
    # Invert the translation vector
    tvec_inv = -np.dot(R_inv, tvec)
    tvec_inv = np.squeeze(tvec_inv)
    # Convert the inverted rotation matrix back to a rotation vector
    rvec_inv, _ = cv2.Rodrigues(R_inv)

    # Define the marker corners in the marker coordinate system
    marker_corners = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

    # Transform the marker corners to the world coordinate system
    world_corners = []
    for corner in marker_corners:
        # Transform the corner
 
        world_point = np.dot(R_inv, corner) + tvec_inv
        world_corners.append(world_point)

    # Output the world coordinates of the marker corners
    for i, world_corner in enumerate(world_corners):
        print(f"Corner {i}: {world_corner}")

    return world_corners


    
