import cv2, yaml
from cv2 import aruco
import numpy as np


table_tags = [
    [163, 0.533, -0.229],
    [168, 0.533, 0],
    [164, 0.533, 0.229],
    [167, 0, -0.229],
    [171, 0, 0],
    [170, 0, 0.229],
    [166, -0.533, -0.229],
    [162, -0.533, 0],
    [172, -0.533, 0.229]
]

def main():

    img = cv2.imread('extrinsic_calibration/extrinsic_calib1.jpg')

    aruco_markersize = 1.5*0.0254

    with open('camera_intrinsic.yaml') as file:
        intrinsics=yaml.load(file, Loader=yaml.FullLoader)
    mtx = np.array(intrinsics['mtx'])
    dist = np.array(intrinsics['dist'])

    aruco_dict = aruco.DICT_6X6_250
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
    aruco_params = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)

    display_img = img.copy()

    detected_tags = dict()

    for corners1, id1 in zip(corners,ids):
                
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners1, aruco_markersize, mtx, dist)

        
        display_img = cv2.aruco.drawAxis(display_img, mtx, dist, rvec, tvec, aruco_markersize*0.75)  # Draw Axis
        
        detected_tags[id1[0]] = tvec.flatten()    
        

    # A = markers in table frame
    # B = detected markers in camera frame

    A = []
    B = []
    for t in table_tags:
        A.append(np.array([t[1],t[2],0.0]))
        tvec = detected_tags[t[0]]
        B.append(tvec)

    A = np.asarray(A).T
    B = np.asarray(B).T

    R,p = rigid_transform_3D(A,B)

    R_cam = R.T
    p_cam = -R.T @ p

    print(f"R_cam={R_cam}")
    print(f"p_cam={p_cam}")


    display_img = aruco.drawDetectedMarkers(display_img,corners,ids)

    display_img = cv2.aruco.drawAxis(display_img, mtx, dist, cv2.Rodrigues(R)[0], (p).flatten(), 0.2) 

    dict_file={'R':R_cam.tolist(),'p':p_cam.tolist()}
    with open('camera_extrinsic.yaml','w') as file:
        yaml.dump(dict_file,file)


    cv2.imshow("img",display_img)
    cv2.waitKey()
    cv2.destroyAllWindows()

def rigid_transform_3D(A, B):
    # This function is taken from https://github.com/nghiaho12/rigid_transform_3D.git
    # Further information: http://nghiaho.com/?page_id=671
    
    # Input: expects 3xN matrix of points
    # Returns R,t
    # R = 3x3 rotation matrix
    # t = 3x1 column vector
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t


if __name__ == "__main__":
    main()