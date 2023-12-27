#!/usr/bin/env python


# Ros
import rospy
from sensor_msgs.msg import PointCloud2, PointField

import sys
import threading
import os
import termios
import cv2
import numpy as np

# 3D
import open3d as o3d
import copy

from ctypes import *

sys.path.append("MvImport")
from MvCameraControl_class import *

global g_numArray
g_numArray = None


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)



def resize_percent(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def Mono_numpy(data, nWidth, nHeight):
    data_ = np.frombuffer(data, count=int(nWidth * nHeight), dtype=np.uint8, offset=0)
    data_mono_arr = data_.reshape(nWidth, nHeight)
    numArray = np.zeros([nWidth, nHeight, 1], "uint8")
    numArray[:, :, 0] = data_mono_arr
    return numArray

def Color_numpy(data, nWidth, nHeight):
    data_ = np.frombuffer(data, count=int(nWidth * nHeight * 3), dtype=np.uint8, offset=0)
    data_r = data_[0:nWidth * nHeight * 3:3]
    data_g = data_[1:nWidth * nHeight * 3:3]
    data_b = data_[2:nWidth * nHeight * 3:3]

    data_r_arr = data_r.reshape(nWidth, nHeight)
    data_g_arr = data_g.reshape(nWidth, nHeight)
    data_b_arr = data_b.reshape(nWidth, nHeight)
    numArray = np.zeros([nWidth, nHeight, 3], "uint8")

    numArray[:, :, 0] = data_r_arr
    numArray[:, :, 1] = data_g_arr
    numArray[:, :, 2] = data_b_arr
    return numArray

def press_any_key_exit():
	fd = sys.stdin.fileno()
	old_ttyinfo = termios.tcgetattr(fd)
	new_ttyinfo = old_ttyinfo[:]
	new_ttyinfo[3] &= ~termios.ICANON
	new_ttyinfo[3] &= ~termios.ECHO
	#sys.stdout.write(msg)
	#sys.stdout.flush()
	termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
	try:
		os.read(fd, 7)
	except:
		pass
	finally:
		termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)
	
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
	
	def open3d_to_ros(self,point_cloud):
		# Convert Open3D PointCloud to numpy array
		points = np.asarray(point_cloud.points)
		colors = np.asarray(point_cloud.colors)
		# Create ROS PointCloud2 message
		ros_msg = PointCloud2()
		ros_msg.header.stamp = rospy.Time.now()
		ros_msg.header.frame_id = "map"
		ros_msg.height = 1
		ros_msg.width = len(points)
		ros_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
		ros_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
		ros_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
		ros_msg.is_bigendian = False
		ros_msg.point_step = 12
		ros_msg.row_step = ros_msg.point_step * len(points)
		ros_msg.is_dense = True
		# Convert numpy arrays to binary data and add to ROS message
		ros_msg.data = points.astype(np.float32).tostring()
		return ros_msg


if __name__ == "__main__":
	rospy.init_node("hikrobot")
	pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

	SDKVersion = MvCamera.MV_CC_GetSDKVersion()
	print ("SDKVersion[0x%x]" % SDKVersion)

	deviceList = MV_CC_DEVICE_INFO_LIST()
	tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
	
	#en:Enum device
	ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
	if ret != 0:
		print ("enum devices fail! ret[0x%x]" % ret)
		sys.exit()

	if deviceList.nDeviceNum == 0:
		print ("find no device!")
		sys.exit()

	print ("Find %d devices!" % deviceList.nDeviceNum)

	for i in range(0, deviceList.nDeviceNum):
		mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
		if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
			print ("\ngige device: [%d]" % i)
			strModeName = ""
			for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
				strModeName = strModeName + chr(per)
			print ("device model name: %s" % strModeName)

			nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
			nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
			nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
			nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
			print ("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
		elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
			print ("\nu3v device: [%d]" % i)
			strModeName = ""
			for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
				if per == 0:
					break
				strModeName = strModeName + chr(per)
			print ("device model name: %s" % strModeName)

			strSerialNumber = ""
			for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
				if per == 0:
					break
				strSerialNumber = strSerialNumber + chr(per)
			print ("user serial number: %s" % strSerialNumber)

	if sys.version >= '3':
		nConnectionNum = input("please input the number of the device to connect:")
	else:
		nConnectionNum = raw_input("please input the number of the device to connect:")

	if int(nConnectionNum) >= deviceList.nDeviceNum:
		print ("intput error!")
		sys.exit()

	#en:Creat Camera Object
	cam = MvCamera()
	
	#en:Select device and create handle
	stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

	ret = cam.MV_CC_CreateHandle(stDeviceList)
	if ret != 0:
		print ("create handle fail! ret[0x%x]" % ret)
		sys.exit()

	#en:Open device
	ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
	if ret != 0:
		print ("open device fail! ret[0x%x]" % ret)
		sys.exit()
	
	#en:Detection network optimal package size(It only works for the GigE camera)
	if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
		nPacketSize = cam.MV_CC_GetOptimalPacketSize()
		if int(nPacketSize) > 0:
			ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize",nPacketSize)
			if ret != 0:
				print ("Warning: Set Packet Size fail! ret[0x%x]" % ret)
		else:
			print ("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

	#en:Set trigger mode as off
	ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
	if ret != 0:
		print ("set trigger mode fail! ret[0x%x]" % ret)
		sys.exit()

	#en:Get payload size
	stParam =  MVCC_INTVALUE()
	memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
	
	ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
	if ret != 0:
		print ("get payload size fail! ret[0x%x]" % ret)
		sys.exit()
	nPayloadSize = stParam.nCurValue

	#en:Start grab image
	ret = cam.MV_CC_StartGrabbing()
	if ret != 0:
		print ("start grabbing fail! ret[0x%x]" % ret)
		sys.exit()

	stOutFrame = MV_FRAME_OUT()
	memset(byref(stOutFrame), 0, sizeof(stOutFrame))
	num = 1

	hik = hikrobot()

	while True:
		ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
		if None != stOutFrame.pBufAddr and 0 == ret:
			print("get one frame: Width[%d], Height[%d], nFrameNum[%d],nFrameLen[%d]" % (
			stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum,stOutFrame.stFrameInfo.nFrameLen))
			
			buf_cache = (c_ubyte * stOutFrame.stFrameInfo.nFrameLen)()
					
			memmove(byref(buf_cache), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nFrameLen)
			if PixelType_Gvsp_Mono8 == stOutFrame.stFrameInfo.enPixelType:
				g_numArray = Mono_numpy(buf_cache,stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight)
				g_numArray = g_numArray.reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 1)
			elif PixelType_Gvsp_RGB8_Packed == stOutFrame.stFrameInfo.enPixelType:
				g_numArray = Color_numpy(buf_cache,stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight)
				g_numArray = g_numArray.reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 3)
			else:
				print("Not Support");
				sys.exit()

			# cv2.imwrite("./Image_Mat.bmp",g_numArray)
			# print(f"g_numArray : {g_numArray.shape}")
			# print(f"g_numArray : {g_numArray[0,0,0]}")
			# print(f"g_numArray : {np.max(g_numArray)}")

			cv2.imshow('g_numArray', resize_percent(g_numArray,50))

			pcd = hik.img2pcd(g_numArray)
			ros_pcd = hik.open3d_to_ros(pcd)
			ros_pcd.header.stamp = rospy.Time.now()
			pub.publish(ros_pcd)

			nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)
			

		else:
			print("no data[0x%x]" % ret)
		
		waitkeyboard = cv2.waitKey(1)

		if waitkeyboard & 0xFF==ord('q'):
			cv2.destroyAllWindows() 
			break
		
		if waitkeyboard & 0xFF==ord('s'):
			cv2.imwrite(f"./Image_Mat{num}.bmp",g_numArray)
			num +=1
	
		
	print ("press a key to stop grabbing.")
	press_any_key_exit()

	#en:Stop grab image
	ret = cam.MV_CC_StopGrabbing()
	if ret != 0:
		print ("stop grabbing fail! ret[0x%x]" % ret)
		del data_buf
		sys.exit()

	#Close device
	ret = cam.MV_CC_CloseDevice()
	if ret != 0:
		print ("close deivce fail! ret[0x%x]" % ret)
		del data_buf
		sys.exit()

	#Destroy handle
	ret = cam.MV_CC_DestroyHandle()
	if ret != 0:
		print ("destroy handle fail! ret[0x%x]" % ret)
		del data_buf
		sys.exit()
