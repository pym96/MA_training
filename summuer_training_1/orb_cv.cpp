#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <vector>
#include <fmt/color.h>

using namespace cv;


int main(int argc, char** argv){

    if(argc != 3){
        fmt::print(fg(fmt::color::red),"Usage: feature_extraction img1 img2\n");
        return 1;
    }

    
    // -- Loading image here
    Mat img_1 = imread(argv[1],IMREAD_COLOR);
    Mat img_2 = imread(argv[2],IMREAD_COLOR);

    assert(img_1.data != nullptr && img_2.data != nullptr);

    // -- Initialization
    std::vector<KeyPoint> keypoint_1, keypoint_2;
    Mat description_1, description_2;

    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptior = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    
    // -- 1. Detecting the corner point
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    detector->detect(img_1,keypoint_1);
    detector->detect(img_2,keypoint_2);

    // -- 2.Computing the BRIEF descripto based on the corner point pose
    descriptior->compute(img_1,keypoint_1,description_1);
    descriptior->compute(img_2,keypoint_2,description_2);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used 
                = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    
    
    fmt::print(fg(fmt::color::green),"Extract ORB point costed time is  {} second\n",time_used.count());
    

    Mat outimg_1;
    drawKeypoints(img_1, keypoint_1, outimg_1);
    imshow("ORB features",outimg_1);
    

    waitKey(0);

    return 0;
}