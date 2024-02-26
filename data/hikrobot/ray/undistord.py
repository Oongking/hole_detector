import numpy as np
import cv2

hikrobot_camera_matrix = np.array([[1263.58549252,    0.,          969.807197  ],
                                [   0.,         1265.2997817,   661.36108893],
                                [   0.,            0.,            1.        ]])
hikrobot_distortion_coefficients = np.array([[-0.08188593,  0.15785128,  0.0029234,   0.00036161, -0.12104376]])

img = cv2.imread('/home/oongking/Research_ws/src/hole_detector/data/hikrobot/ray/ray5_1.bmp')
h,  w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(hikrobot_camera_matrix, hikrobot_distortion_coefficients, (w,h), 1, (w,h))

# undistort
dst = cv2.undistort(img, hikrobot_camera_matrix, hikrobot_distortion_coefficients, None, newcameramtx)
print(f"dst : {dst.shape}")#dst : (1200, 1920, 3)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
print(f"dst : {dst.shape}")#dst : (1172, 1877, 3)

print(f"hikrobot_camera_matrix : {hikrobot_camera_matrix}")
print(f"newcameramtx : {newcameramtx}")
while True:
    cv2.imshow('img', img)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('q'):
        break

while True:
    cv2.imshow('img', dst)
    waitkeyboard = cv2.waitKey(1)

    if waitkeyboard & 0xFF==ord('q'):
        break


