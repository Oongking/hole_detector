 /*
  * OpenCV Example using ROS and CPP
  */

 // Include the ROS library
#include <ros/ros.h>

#include <string>

#include "open3d/Open3D.h"
//  have in open3d
// #include <eigen3/Eigen/Eigenvalues>
// #include <eigen3/Eigen/Sparse>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>


#include <iostream>

// Include opencv2
#include <opencv2/opencv.hpp>


// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> // Required for converting ROS PC2 to PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <time.h>

using Eigen::MatrixXd;
cv::Mat input_img,cb_img;

cv::Mat resizePercent(const cv::Mat& img, double scalePercent) {
    int width = static_cast<int>(img.cols * scalePercent / 100);
    int height = static_cast<int>(img.rows * scalePercent / 100);
    cv::Size newSize(width, height);

    cv::Mat resized;
    cv::resize(img, resized, newSize, 0, 0, cv::INTER_AREA);

    return resized;
}

std::vector<double> find_point_on_plane(const std::vector<double>& pOrigin, const std::vector<double>& pNormal, double dis) {
    std::cout << "pOrigin : ";
    for (const auto& val : pOrigin)
        std::cout << val << " ";
    std::cout << std::endl;

    std::cout << "pNormal : ";
    for (const auto& val : pNormal)
        std::cout << val << " ";
    std::cout << std::endl;

    double y = (pNormal[2] * dis - pNormal[0] * pOrigin[0] - pNormal[1] * pOrigin[1] - pNormal[2] * pOrigin[2]) / (-pNormal[1]);
    std::cout << "y : " << y << std::endl;

    std::vector<double> point = {0, y, dis};
    
    return point;
}

int main(int argc, char** argv)
{
   // Initialize the ROS Node "roscpp_opencv"
    ros::init(argc, argv, "line_scan");
    ros::NodeHandle nh;

   // Instantiate the ROS Node Handler as nh
    // std::cout << "while" << std::endl;

    ros::Rate r(10);

    clock_t clkStart;
    clock_t clkFinish;

    // Define camera intrinsic parameters
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1263.58549252,    0.,          969.807197, 
                                                        0.,         1265.2997817,   661.36108893, 
                                                        0.,           0.,           1.        ); // Example camera matrix

    cv::Mat ray_plane_origin = (cv::Mat_<double>(1,3) << 0.00421182, -0.00756944,  0.80403207); // Example ray plane (Origin)
    cv::Mat ray_plane_normal = (cv::Mat_<double>(1,3) << 0.00265547,  0.95242381, -0.30476522); // Example ray plane (Normal)

    // Define distortion coefficients (typically zero for most cameras)
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1, CV_64F) << -0.08188593,  0.15785128,  0.0029234,   0.00036161, -0.12104376); // Example distortion coefficients

    // Define rotation vector and translation vector (extrinsic parameters)
    cv::Mat rvec = (cv::Mat_<double>(3, 1)); // Example rotation vector (in radians)
    cv::Mat rmat = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0); // Example translation vector
    // input_img=cv::imread("/home/oongking/Research_ws/src/hole_detector/script/hikrobot/Image_Mat1.bmp");
    input_img=cv::imread("/home/oongking/Research_ws/src/hole_detector/data/onsite/laser_calibrate/1.png");
    //reads the input image
    cv::namedWindow("test",cv::WINDOW_AUTOSIZE);

    cv::cvtColor(input_img,cb_img,cv::COLOR_BGR2GRAY);


    while(!ros::isShuttingDown()){
        ros::spinOnce();
        if (!cb_img.empty()){
            clkStart = clock();

            // std::cout << "Width : " << cb_img.size().width << std::endl;
            // std::cout << "Height: " << cb_img.size().height << std::endl;
            
            // cv::imshow("test", resizePercent(cb_img,50));
            // cv::waitKey(0);

            cv::Mat thres_img;
            cv::threshold(cb_img,thres_img,127,255,cv::THRESH_BINARY);
            

            std::vector<cv::Point2d> imagePoints;
            std::vector<double> result;

            // min
            result = find_point_on_plane(ray_plane_origin, ray_plane_normal, 0.6);

            std::cout << "Resulting point: ";
            for (const auto& val : result)
                std::cout << val << " ";
            std::cout << std::endl;
            
            cv::projectPoints(result, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cameraMatrix, distCoeffs, imagePoints);
            std::cout << "imagePoints x : "<< int(imagePoints[0].x) << "y : "<< int(imagePoints[0].y) <<std::endl;

            for (int y = 0; y < int(imagePoints[0].y); ++y) {
                // Fill each row with the specified color
                uchar* rowPtr = thres_img.ptr<uchar>(y);
                for (int x = 0; x < thres_img.cols; ++x) {
                    rowPtr[x] = 0;
                }
            }

            // max
            result = find_point_on_plane(ray_plane_origin, ray_plane_normal, 2.5);

            std::cout << "Resulting point: ";
            for (const auto& val : result)
                std::cout << val << " ";
            std::cout << std::endl;
            
            cv::projectPoints(result, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cameraMatrix, distCoeffs, imagePoints);
            std::cout << "imagePoints x : "<< int(imagePoints[0].x) << "y : "<< int(imagePoints[0].y) <<std::endl;
            
            for (int y = 1200; y > int(imagePoints[0].y); --y) {
                // Fill each row with the specified color
                uchar* rowPtr = thres_img.ptr<uchar>(y);
                for (int x = 0; x < thres_img.cols; ++x) {
                    rowPtr[x] = 0;
                }
            }

            float a = 5.0f;
            double b = static_cast<double>(a);
            std::cout << "Double value: " << b << std::endl;

            cv::imshow("test", resizePercent(thres_img,50));
            cv::waitKey(0);

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
                double area = cv::contourArea(contour);

                // std::cout << " Contour Area " << area << std::endl;

                for (int i = 0; i < mask.cols; ++i) {
                    // std::cout << "i: " << i << std::endl;

                    cv::Mat column = mask.col(i);
                    cv::Mat nonZeroCoords;
                    cv::findNonZero(column, nonZeroCoords);

                    if (!nonZeroCoords.empty() && area < 6000) {

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

                // cv::imshow("test", resizePercent(mask_final,50));
                // cv::waitKey(0);
            }

            

            // cv::imshow("test", resizePercent(mask_final,50));
            // cv::waitKey(0);

            cv::Mat laserPts;
            cv::findNonZero(mask_final, laserPts);

            if (!laserPts.empty()){
                for (int i = 0; i < laserPts.rows; ++i) {
                    for (int j = 0; j < laserPts.cols; ++j) {
                    cv::Vec2i& ray_index = laserPts.at<cv::Vec2i>(i, j);
                    // std::cout << "ray_index: " << ray_index << std::endl;
                    cv::circle(input_img,ray_index,1,(0,255,255));
                    }
                }
            }

            // std::cout << " laserPts.rows: " <<  laserPts.rows << std::endl;
            // std::cout << "laserPts.cols: " << laserPts.cols << std::endl;
            // cv::imshow("test", resizePercent(input_img,50));
            // cv::waitKey(0);
            
            // cv::Mat homoImgPoints = cv::Mat::zeros(laserPts.rows, laserPts.cols, CV_64FC3);
            
            // for (int i = 0; i < homoImgPoints.rows; ++i) {
            //     for (int j = 0; j < homoImgPoints.cols; ++j) {
            //         cv::Vec2i& ray_index = laserPts.at<cv::Vec2i>(i, j);
            //         // Assigning arbitrary values (e.g., increasing values for demonstration purposes)
            //         homoImgPoints.at<cv::Vec3d>(i, j) = cv::Vec3d(ray_index[0], ray_index[1], 1);
            //     }
            // }

            // cv::Mat invert_cameraMatrix;

            // // std::cout << "homoImgPoints in: " << homoImgPoints << std::endl;
            // // std::cout << "homoImgPoints .size(): " << homoImgPoints.size() << std::endl;

            // // Find the inverse of the matrix
            // bool invertible = cv::invert(cameraMatrix, invert_cameraMatrix);

            // // std::cout << "invertible : " << invertible << std::endl;

            // cv::Mat rays = cv::Mat::zeros(homoImgPoints.rows, homoImgPoints.cols, CV_64FC3);

            // for (int i = 0; i < homoImgPoints.rows; ++i) {
            //     for (int j = 0; j < homoImgPoints.cols; ++j) {

            //         cv::Vec3d& homo_vector = homoImgPoints.at<cv::Vec3d>(i, j);


            //         // std::cout << "rays  "<< std::endl;
            //         // std::cout << "invert_cameraMatrix  " << invert_cameraMatrix << std::endl;

            //         cv::Mat ray_vec = (cv::Mat_<double>(3, 1) << homo_vector[0],homo_vector[1], homo_vector[2]);
                
            //         // std::cout << "ray_vec : " << ray_vec << std::endl;

            //         cv::Mat result = invert_cameraMatrix * ray_vec;

            //         // std::cout << "result : " << result << std::endl;
            //         // std::cout << "result.at : " << result.at<double>(0) << std::endl;

            //         double* doubleData = result.ptr<double>();
            //         // Assigning arbitrary values (e.g., increasing values for demonstration purposes)
                    
            //         rays.at<cv::Vec3d>(i, j) = cv::Vec3d(doubleData[0], 
            //                                             doubleData[1], 
            //                                             doubleData[2]);
            //         // std::cout << "rays.at<cv::Vec3f>(i, j) : " << rays.at<cv::Vec3f>(i, j) << std::endl;
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



            // std::cout << "rays 0: " << rays.at<cv::Vec3d>(0, 0)<< std::endl;
            // std::cout << "rays in.size(): " << rays.size() << std::endl;

            double dotProduct_O_N = cv::Mat(ray_plane_origin * ray_plane_normal.t()).at<double>(0, 0);
            
            // std::cout << "dotProduct_O_N: " << dotProduct_O_N << std::endl;
            open3d::geometry::PointCloud laser_pcd;

            for (int i = 0; i < rays.rows; ++i) {
                cv::Vec3d& ray = rays.at<cv::Vec3d>(i, 0);
                // std::cout << "ray: " << ray << std::endl;
                // std::cout << "ray_plane_normal.t(): " << ray_plane_normal.t() << std::endl;

                cv::Mat ray_mul = (cv::Mat_<double>(1, 3) << ray[0],ray[1], ray[2]);
                double dotProduct_R_N = cv::Mat(ray_mul * ray_plane_normal.t()).at<double>(0, 0);
                // std::cout << "dotProduct_R_N: " << dotProduct_R_N << std::endl;
        
                // rays.at<cv::Vec3d>(i, 0) = ray*(dotProduct_O_N/dotProduct_R_N);

                // std::cout << "ray: " << ray << std::endl;
                ray *= (dotProduct_O_N/dotProduct_R_N);
                // std::cout << "(dotProduct_O_N/dotProduct_R_N): " << (dotProduct_O_N/dotProduct_R_N) << std::endl;

                // std::cout << "ray2: " << ray << std::endl;

                laser_pcd.points_.push_back(Eigen::Vector3d(ray[0],ray[1], ray[2]));
                // std::cout << "ray: " << ray << std::endl;
                // std::cout << std::setprecision(15) << "dotProduct_O_N/dotProduct_R_N: " << (dotProduct_O_N/dotProduct_R_N) << std::endl;
                
            }

            // std::cout << "rays out: " << rays << std::endl;
            // std::cout << "rays out.size(): " << rays.size() << std::endl;

            // std::cout << "END" << std::endl;
            
            // std::cout << " == finish == "<< std::endl;
            clkFinish = clock();
            std::cout <<" Run time : "<< (double)(clkFinish - clkStart)/CLOCKS_PER_SEC<< std::endl;

            std::cout << " == display == "<< std::endl;
            std::shared_ptr<open3d::geometry::TriangleMesh> Realcoor = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
            open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(laser_pcd),Realcoor});
            std::cout << " == laser_pcd.points_ == " << laser_pcd.points_[0] << std::endl;
            
            cb_img = cv::Mat();
        
            break;
        }
        r.sleep();
    }


    cv::destroyWindow("test");
    // Program succesful
    return 0;
 }