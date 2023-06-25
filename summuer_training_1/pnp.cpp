#include "ros/ros.h"
#include "ros/console.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_estimation_test");
    ros::NodeHandle nh;

	// Read the camera calibration parameters
	cv::Mat cameraMatrix;
	cv::Mat cameraDistCoeffs;
	cv::FileStorage fs("/home/macs/.ros/camera_info/calib.xml", cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        ROS_ERROR("unable to load camera matrix");
    }
    fs["Intrinsic"] >> cameraMatrix;
    fs["Distortion"] >> cameraDistCoeffs;
    std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
    std::cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << std::endl << std::endl;
	cv::Matx33d cMatrix(cameraMatrix);
	
    // capture a image from video
    cv::VideoCapture cap(0);
    if(!cap.isOpened())
    {
        ROS_ERROR("failed to open vedio stream form port 0");
    }

    // Chesseboard corners in 3D coodinate.
    cv::Size boardSize(7,5);
    std::vector<cv::Point3f> objectPoints;
    for(int i=0; i<boardSize.height; i++)
    {
        for(int j=0; j<boardSize.width; j++)
        {
            objectPoints.push_back(cv::Point3f(i*0.03,j*0.03,0.0f));
        }
    }
    // creat a viz window
    cv::viz::Viz3d visualizer("Viz window");
    visualizer.setBackgroundColor(cv::viz::Color::white());
    cv::namedWindow("captured image",1);
    while(ros::ok())
    {
        cv::Mat frame;
        cv::Mat rvec, tvec;
        cap >> frame;    //grabe an image from video
        if(frame.empty())
        {
            ROS_ERROR("failed to grabe an image from video");
        }
        ROS_INFO("image grabed");
        
        // get chesseboard corners from the image
        std::vector<cv::Point2f> imageCorners;
        bool patternfound = cv::findChessboardCorners(frame,boardSize,imageCorners,cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_FAST_CHECK);
        if(patternfound)
        {
            /// Construct the scene
            // Create a virtual camera
            cv::viz::WCameraPosition cam(cMatrix,  // matrix of intrinsics
                                        frame,    // image displayed on the plane
                                        0.1,     // scale factor
                                        cv::viz::Color::black());
            // Create a virtual bench from cuboids
            cv::viz::WCube plane1(cv::Point3f(0.0, 0, 0.0), 
                                cv::Point3f(0.12, 0.18, -0.02), 
                                true,  // show wire frame 
                                cv::viz::Color::blue());
            plane1.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
            // Get the camera pose from 3D/2D points
            cv::solvePnP(objectPoints, imageCorners,      // corresponding 3D/2D pts 
                    cameraMatrix, cameraDistCoeffs, // calibration 
                    rvec, tvec);                    // output pose
            std::cout << " rvec: " << rvec.rows << "x" << rvec.cols << std::endl;
            std::cout << " tvec: " << tvec.rows << "x" << tvec.cols << std::endl;
            cv::drawChessboardCorners(frame,boardSize,imageCorners,patternfound);
            // Add the virtual objects to the environment
            visualizer.showWidget("top", plane1);
            visualizer.showWidget("Camera", cam);

            cv::Mat rotation;
            // convert vector-3 rotation
            // to a 3x3 rotation matrix
            cv::Rodrigues(rvec, rotation);

            // Move the bench	
            cv::Affine3d pose(rotation, tvec);
            visualizer.setWidgetPose("Camera", pose.inv());
        }
        cv::imshow("captured image",frame);
        int i=cv::waitKey(33.3);
        visualizer.spinOnce(1,true);
    }

    return 0;
}