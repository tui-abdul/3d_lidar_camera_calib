import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

'''
cap = cv2.VideoCapture(0) 


cMat =[ [1449.40738,    0.     ,  872.37441] ,
       [     0.     , 1451.89188,  588.19162] ,
       [     0.     ,    0.     ,    1.     ]]
dcoeff=[  [-0.026885, -0.136461, 0.000475, 0.003409, 0.000000]]

cMat1 = [[ 89.27210282,   0. ,        286.68123882],
 [  0.      ,    95.0341892 , 239.30344084],
 [  0.       ,    0.    ,       1.        ]]

dcoeff1 = [[ 0.00306865, -0.00118168, -0.00588176,  0.01020636 , 0.00010566]]
dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
dt = cv2.aruco.DetectorParameters()
'''
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


    

''''
while True:
    ret, frame = cap.read()
    detector = cv2.aruco.ArucoDetector(dict, dt)
    corners, ids, _ = detector.detectMarkers(frame)

    #if ids is not None and ids.size > 0:
    #id = ids[0]
    #corner = corners[0][0]
    if corners:
        rvec, tvec, _ = my_estimatePoseSingleMarkers(corners[0], 5, cMat, dcoeff)
        #print(tvec[0])
        dist = np.sqrt(tvec[0][0]**2 + tvec [0][1] **2 + tvec [0][2]**2)#np.linalg.norm(tvec)
        cv2.aruco.drawDetectedMarkers(frame, corners)
        cv2.putText(frame, f"Distance: {dist} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

if __name__ == '__main__':
    with open('calibrationdata/ost.yaml', "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)

    matrix_coefficients = calib_data["camera_matrix"]["data"]
    distortion_coefficients=calib_data["distortion_coefficients"]["data"]

    track(np.array(matrix_coefficients).reshape(3,3),np.array(distortion_coefficients))
'''