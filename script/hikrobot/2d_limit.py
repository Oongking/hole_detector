import numpy as np
import cv2
import open3d as o3d
import copy

def resize_percent(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

class hikrobot:
	def __init__(self):
		self.camera_matrix = np.array([[1263.58549252,    0.,          969.807197  ],
                                [   0.,         1265.2997817,   661.36108893],
                                [   0.,            0.,            1.        ]])
		self.distortion_coefficients = np.array([[-0.08188593,  0.15785128,  0.0029234,   0.00036161, -0.12104376]])

		self.ray_plane = [np.array([ 0.00421182, -0.00756944,  0.80403207]),np.array([ 0.00265547,  0.95242381, -0.30476522])]

	def createRays(self,pts, K_inv):
		"""
		Transforms an array of points into an array of 3D rays, using the inverse intrinsics K_inv.
		"""
		# We simply need to multiply per K_inv, this way we get a 3D direction.
		# We can get the points of the ray multiplying the direction with a scalar.
		return [np.matmul(K_inv, p) for p in pts]

	def linePlaneIntersection(self,plane, rayDir):
		"""
		Calculate the 3D intersection between a plane and a ray, returning a 3D point.
		"""
		pOrigin, pNormal = plane
		# print(f"pOrigin : {pOrigin.shape}, {pOrigin}")
		# print(f"pNormal : {pNormal.shape}, {pNormal}")
		# print(f"rayDir : {rayDir.shape}")
		d = np.dot(pOrigin, pNormal) / np.dot(rayDir, pNormal)
		return rayDir * d

	def img2pcd(self,img):
		pcd = o3d.geometry.PointCloud()
		# process_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		process_img = copy.deepcopy(img)
		ret,thresh1 = cv2.threshold(process_img,127,255,cv2.THRESH_BINARY)
		kernel4 = np.ones((4, 4), np.uint8)

		final = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel4)

		contours, hierarchy = cv2.findContours(final, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE )

		# print(f"contours : {len(contours)}")
		mask_final = np.zeros(final.shape)
		for contour in contours:
			mask = np.zeros(final.shape)
			cv2.fillPoly(mask, [contour], (255))
			for i in range(mask.shape[1]):
				laserPts = cv2.findNonZero(mask[:,i])
				if laserPts is not None:
					# print(f"laserPts : {laserPts}")
					average = np.average(laserPts,axis = 0).reshape(-1)
					# print(f"average : {average}")
					mask_final[int(average[1]),i] = 255

		laserPts = cv2.findNonZero(mask_final)
		if laserPts is not None:
			homoImgPoints = np.hstack(
				(laserPts[:, 0], np.ones(laserPts.shape[0]).reshape(-1, 1),))
			
			rays = self.createRays(homoImgPoints, np.linalg.inv(self.camera_matrix))
			points3D = [self.linePlaneIntersection(self.ray_plane, ray) for ray in rays]
			# print(points3D)
			objPoints = []
			objPoints.extend(points3D)
			pcd.points = o3d.utility.Vector3dVector(np.vstack(objPoints).astype(np.float64))

		return pcd
	def find_point_on_plane(self,dis):
		pOrigin, pNormal = self.ray_plane
		print(f"pOrigin : {pOrigin}")
		print(f"pNormal : {pNormal}")
		y = (pNormal[2]*dis-pNormal[0]*pOrigin[0]-pNormal[1]*pOrigin[1]-pNormal[2]*pOrigin[2])/(-pNormal[1])
		print(f"y : {y}")
		point = [0,y,dis]
		return point

img = cv2.imread("/home/oongking/Research_ws/src/hole_detector/data/PIC/image.bmp")

cv2.imshow("test",img)
# cv2.waitKey(0)

hik = hikrobot()
for i in range(100):
    point = hik.find_point_on_plane(0.2+i*0.05)

    imagePoints, jacobian = cv2.projectPoints(np.array([[point]]),cv2.Rodrigues(np.eye(3))[0],np.array([0.,0.,0.]),hik.camera_matrix,hik.distortion_coefficients)
    imagePoints = np.array(imagePoints,dtype=np.int32).reshape(-1)
    print(f"imagePoints : {imagePoints}")

    cv2.circle(img, imagePoints, 1, [0,255,0], 1)

cv2.imshow("test",resize_percent(img,50))
cv2.waitKey(0)

