#include <vector>
#include <fmt/color.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


using namespace cv;


void pose_estimation_2d2d(std::vector<KeyPoint>, 
                          std::vector<KeyPoint>, 
                          std::vector<DMatch>,
                          Mat&,
                          Mat&);

int main(int argc, char** argv){

    if(argc != 3){
        fmt::print(fg(fmt::color::red),"Current input is not valid, argc = {}", argc);
        return -1;
    }

    Mat img_1 = imread(argv[1],IMREAD_COLOR);
    Mat img_2 = imread(argv[2],IMREAD_COLOR);

    assert(img_1.data && img_2.data && "Can't load image");

 
    std::vector<KeyPoint> keypoint_1, keypoint_2;
    std::vector<DMatch> matches;
    
    // find_feature_matches(img_1,img_2,keypoint_1,keypoint_2,matches);
    

    


    return 0;
}

void pose_estimation_2d2d(std::vector<KeyPoint> key_point1, 
                          std::vector<KeyPoint> key_point2, 
                          std::vector<DMatch> matcher,
                          Mat& R,
                          Mat& T)
{
    // Camera inner parameter
    Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    
    // -- Converting matching feature points into 2d pixel array
    std::vector<Point2f> points1;
    std::vector<Point2f> points2;
    
    for(int i = 0; i < (int) matcher.size(); ++i){
        points1.push_back(key_point1[matcher[i].queryIdx].pt);
        points2.push_back(key_point2[matcher[i].queryIdx].pt);
    }

    // -- Caculating the basic matrix, 2 stands for the 8 points algorithm
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1,points2,2);

    std::cout << "Current fundamental matrix is: \n" << fundamental_matrix << '\n';

    // -- Caculating the nature matrix 
    Point2d principal_point(325.1,249.7); // Camera's light core, TUM standard
    double focal_length = 521; // Camera's focal length, TUM standard

    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    std::cout << "Current essential matrix is: \n" << essential_matrix << '\n';

    // -- Recovering rotation matrix and transition info from essential matrix
    recoverPose(essential_matrix, points1, points2, R, T, focal_length, principal_point);
    std::cout << "Current rotation matrix is: \n" << R << '\n';
    std::cout << "Current transition info is: \n" << T << '\n';
}