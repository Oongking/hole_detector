import cv2
import cv2.aruco as aruco
import numpy as np
import os

width=7
height=6
square_size = 0.030

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((height*width, 3), np.float32)
objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

objp = objp * square_size

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

for i in range(1,21):

    print(f"pic : {i}")
    img = cv2.imread(f'/home/oongking/Research_ws/src/hole_detector/data/hikrobot/calibration/hik{i}.bmp')
    
    # cv2.imshow('img',img)
    # cv2.waitKey(0)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # cv2.imshow('gray',gray)
    # cv2.waitKey(0)
    gray = cv2.bitwise_not(gray)
    # cv2.imshow('gray',gray)
    # cv2.waitKey(0)
    
    

    ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
    # print("corners : ",corners)
    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(500)
        
        cv2.imwrite(f'/home/oongking/Research_ws/src/hole_detector/data/hikrobot/calibration/hik_calibrate_zividchessboard{i}.png',img)


cv2.imshow('img',img)

cv2.waitKey(0)
# print("objpoints : ",objpoints)
# print("imgpoints : ",imgpoints)
np.set_printoptions(suppress=True)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# print("ret : ", ret)
print("mtx : ", mtx)
print("dist : ", dist)
# print("rvecs : ", rvecs)
# print("tvecs : ", tvecs)


f = open("/home/oongking/Research_ws/src/hole_detector/data/hikrobot/calibration/HikZividCamMatrix.txt","w+")
f.write("mtx : ")
f.write(str(mtx))
f.write('\n')
f.write("dist : ")
f.write(str(dist))
f.write('\n')

# f.write("ret : ")
# f.write(str(ret))
# f.write('\n')
# f.write("rvecs : ")
# f.write(str(rvecs))
# f.write('\n')
# f.write("tvecs : ")
# f.write(str(tvecs))


