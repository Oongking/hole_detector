
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include "open3d/Open3D.h"
#include <pcl_conversions/pcl_conversions.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program

// Define camera intrinsic parameters
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1263.58549252,    0.,          969.807197, 
                                                    0.,         1265.2997817,   661.36108893, 
                                                    0.,           0.,           1.        ); // Example camera matrix

cv::Mat ray_plane_origin = (cv::Mat_<double>(1,3) << 0.00951941, -0.00696639,  0.80589888); // Example ray plane (Origin)
cv::Mat ray_plane_normal = (cv::Mat_<double>(1,3) << 0.00286011,  0.95256321, -0.30432738); // Example ray plane (Normal)

// Define distortion coefficients (typically zero for most cameras)
cv::Mat distCoeffs = (cv::Mat_<double>(5, 1, CV_64F) << -0.08188593,  0.15785128,  0.0029234,   0.00036161, -0.12104376); // Example distortion coefficients




cv::Mat resizePercent(const cv::Mat& img, double scalePercent) {
    int width = static_cast<int>(img.cols * scalePercent / 100);
    int height = static_cast<int>(img.rows * scalePercent / 100);
    cv::Size newSize(width, height);

    cv::Mat resized;
    cv::resize(img, resized, newSize, 0, 0, cv::INTER_AREA);

    return resized;
}


bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

sensor_msgs::PointCloud2 createPointCloud2Msg(const open3d::geometry::PointCloud& o3d_pc) {
    sensor_msgs::PointCloud2 ros_pc2;

    // Convert Open3D point cloud to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_pc;
    pcl::PointXYZ pcl_point;
    auto points = o3d_pc.points_;

    for (const auto& point : points) {
        pcl_point.x = point[0];
        pcl_point.y = point[1];
        pcl_point.z = point[2];
        pcl_pc.push_back(pcl_point);
    }

    // Convert PCL point cloud to ROS message
    pcl::toROSMsg(pcl_pc, ros_pc2);
    ros_pc2.header.frame_id = "map"; // Replace with your desired frame ID
    ros_pc2.header.stamp = ros::Time::now();

    return ros_pc2;
}


open3d::geometry::PointCloud calculate_pcd(cv::Mat& cb_img, int& param_value){

    cv::Mat imageUndistorted; // Will be the undistorted version of the above image.
    undistort(cb_img, imageUndistorted, cameraMatrix, distCoeffs);

    cv::Mat thres_img;
    cv::threshold(imageUndistorted,thres_img,param_value,255,cv::THRESH_BINARY);
    // cv::imshow("test", resizePercent(thres_img,50));
    // cv::waitKey(0);

    cv::Mat final;
    cv::Mat kernel4 = cv::Mat::ones(4, 4, CV_8U);
    cv::morphologyEx(thres_img,final,cv::MORPH_OPEN,kernel4);
    // cv::imshow("test", resizePercent(final,50));
    // cv::waitKey(0);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( final, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    cv::Mat mask_final = cv::Mat::zeros( final.size(), CV_8UC1 );

    for (auto& contour : contours) {
        cv::Mat mask = cv::Mat::zeros(final.size(), CV_8UC1);
    
        std::vector<cv::Point> contourVec = {contour.begin(), contour.end()};
        std::vector<std::vector<cv::Point>> contoursPoly = {contourVec};
        
        cv::fillPoly(mask, contoursPoly, cv::Scalar(255));
        // cv::imshow("test", resizePercent(mask,50));
        // cv::waitKey(0);

        for (int i = 0; i < mask.cols; ++i) {
            // std::cout << "i: " << i << std::endl;

            cv::Mat column = mask.col(i);
            cv::Mat nonZeroCoords;
            cv::findNonZero(column, nonZeroCoords);

            if (!nonZeroCoords.empty()) {

                // std::cout << "nonZeroCoords: " << nonZeroCoords.size() << std::endl;
                cv::Mat coordinatesMat = cv::Mat(nonZeroCoords).reshape(1);
                // std::cout << "coordinatesMat size: " << coordinatesMat.size() << std::endl;
                // std::cout << "coordinatesMat in: " << coordinatesMat << std::endl;
                // std::cout << "coordinatesMat col: " << coordinatesMat.col(1) << std::endl;
                
                cv::Scalar average = cv::mean(coordinatesMat.col(1));
                // std::cout << "average in: " << average[0] << std::endl;
                int row = static_cast<int>(average[0]);
                // std::cout << "row: " << row << std::endl;
                
                mask_final.at<uchar>(row, i) = 255;
            }
        }

    }

    cv::Mat laserPts;
    cv::findNonZero(mask_final, laserPts);

    // cv::Mat homoImgPoints = cv::Mat::zeros(laserPts.rows, laserPts.cols, CV_64FC3);
        
    // for (int i = 0; i < homoImgPoints.rows; ++i) {
    //     for (int j = 0; j < homoImgPoints.cols; ++j) {
    //         cv::Vec2i& ray_index = laserPts.at<cv::Vec2i>(i, j);
    //         // Assigning arbitrary values (e.g., increasing values for demonstration purposes)
    //         homoImgPoints.at<cv::Vec3d>(i, j) = cv::Vec3d(ray_index[0], ray_index[1], 1);
    //     }
    // }

    // cv::Mat invert_cameraMatrix;

    // bool invertible = cv::invert(cameraMatrix, invert_cameraMatrix);

    // cv::Mat rays = cv::Mat::zeros(homoImgPoints.rows, homoImgPoints.cols, CV_64FC3);

    // for (int i = 0; i < homoImgPoints.rows; ++i) {
    //     for (int j = 0; j < homoImgPoints.cols; ++j) {

    //         cv::Vec3d& homo_vector = homoImgPoints.at<cv::Vec3d>(i, j);

    //         cv::Mat ray_vec = (cv::Mat_<double>(3, 1) << homo_vector[0],homo_vector[1], homo_vector[2]);

    //         cv::Mat result = invert_cameraMatrix * ray_vec;

    //         double* doubleData = result.ptr<double>();

    //         rays.at<cv::Vec3d>(i, j) = cv::Vec3d(doubleData[0], 
    //                                             doubleData[1], 
    //                                             doubleData[2]);
    //     }
    // }


    cv::Mat invert_cameraMatrix;

    bool invertible = cv::invert(cameraMatrix, invert_cameraMatrix);

    cv::Mat rays = cv::Mat::zeros(laserPts.rows, laserPts.cols, CV_64FC3);

    for (int i = 0; i < laserPts.rows; ++i) {
        for (int j = 0; j < laserPts.cols; ++j) {

            cv::Vec2i& ray_index = laserPts.at<cv::Vec2i>(i, j);

            cv::Mat ray_vec = (cv::Mat_<double>(3, 1) << ray_index[0], ray_index[1], 1);

            cv::Mat result = invert_cameraMatrix * ray_vec;

            double* doubleData = result.ptr<double>();

            rays.at<cv::Vec3d>(i, j) = cv::Vec3d(doubleData[0], 
                                                doubleData[1], 
                                                doubleData[2]);
        }
    }



    double dotProduct_O_N = cv::Mat(ray_plane_origin * ray_plane_normal.t()).at<double>(0, 0);
        
    open3d::geometry::PointCloud laser_pcd;

    for (int i = 0; i < rays.rows; ++i) {
        cv::Vec3d& ray = rays.at<cv::Vec3d>(i, 0);


        cv::Mat ray_mul = (cv::Mat_<double>(1, 3) << ray[0],ray[1], ray[2]);
        double dotProduct_R_N = cv::Mat(ray_mul * ray_plane_normal.t()).at<double>(0, 0);
        // std::cout << "dotProduct_R_N: " << dotProduct_R_N << std::endl;

        // rays.at<cv::Vec3d>(i, 0) = ray*(dotProduct_O_N/dotProduct_R_N);

        // std::cout << "ray: " << ray << std::endl;
        ray *= (dotProduct_O_N/dotProduct_R_N);
 

        laser_pcd.points_.push_back(Eigen::Vector3d(ray[0],ray[1], ray[2]));

        
    }

    return laser_pcd;

}

int main(int argc, char** argv)
{
    int nRet = MV_OK;

    void* handle = NULL;

    ros::init(argc, argv, "hikrobot_linescanner");
    ros::NodeHandle nh;

    // Create a publisher for the compressed image
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/hik/image_raw", 1);
    ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/hik/pointcloud2", 1);

    int param_value;
    if (nh.getParam("/Hardware_pcd/threshold_value", param_value)) {
        ROS_INFO("threshold_value: %d", param_value);
    } else {
        ROS_ERROR("Failed to get parameter 'threshold_value'");
        param_value = 127;
    }
    
    do 
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }  
        } 
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Intput camera index: ");
        unsigned int nIndex = 0;
        // scanf("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            break;
        }

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }

        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }






        // get IFloat variable
        MVCC_FLOATVALUE stExposureTime = {0};
        nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
        if (MV_OK == nRet)
        {
            printf("exposure time current value:%f\n", stExposureTime.fCurValue);
            printf("exposure time max value:%f\n", stExposureTime.fMax);
            printf("exposure time min value:%f\n\n", stExposureTime.fMin);
        }
        else
        {
            printf("get exposure time failed! nRet [%x]\n\n", nRet);
        }

        // 设置float型变量
        // set IFloat variable
        float fExposureTime = 0.0f;
        printf("please input the exposure time to set: ");
        if(0 == scanf("%f", &fExposureTime))
        {
            printf("Input Format Error!");
            break;
        }

        nRet = MV_CC_SetFloatValue(handle, "ExposureTime", fExposureTime);
        if (MV_OK == nRet)
        {
            printf("set exposure time OK!\n\n");
        }
        else
        {
            printf("set exposure time failed! nRet [%x]\n\n", nRet);
        }






		
        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }
		
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            break;
        }


        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            break;
        }


        int nRet = MV_OK;

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return NULL;
        }

        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            return NULL;
        }
        unsigned int nDataSize = stParam.nCurValue;

        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(100); // Adjust JPEG quality as needed

        std::cout << std::endl << std::endl << "\n########## Start Hikrobot ##########" << std::endl;
        while(!ros::isShuttingDown())
        {
            nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
            if (nRet == MV_OK)
            {
                // printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", 
                //     stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
                
                cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
                int rows = stImageInfo.nHeight;
                int cols = stImageInfo.nWidth;
                int channels = 1;

                // Create a cv::Mat using the image data
                cv::Mat img(rows, cols, CV_8UC(channels), pData);

                // cv::imshow("test", resizePercent(img,50));
                // cv::waitKey(1);
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
                image_pub.publish(msg);

                open3d::geometry::PointCloud pcd_out = calculate_pcd(img, param_value);
                // std::shared_ptr<open3d::geometry::TriangleMesh> Realcoor = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
                // open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(pcd_out),Realcoor});
                sensor_msgs::PointCloud2 ros_pc2 = createPointCloud2Msg(pcd_out);
                pointcloud_pub.publish(ros_pc2);

            }
            else{
                printf("No data[%x]\n", nRet);
            }
        }

        free(pData);

        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            break;
        }

        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            break;
        }
    } while (0);

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("exit.\n");
    return 0;
}
