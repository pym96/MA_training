// Copyright 2023 Pan Jiaxiang.
// Licensed under the MIT License.

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
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

    resize(img_1, img_1, Size2d(600,480),INTER_LINEAR);
    resize(img_2, img_2, Size2d(600,480),INTER_LINEAR);


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

    // -- 3.Marching the BRIEF descriptor point based on the comparision of two images
    // -- and get the hamming distance
    std::vector<DMatch> matches;
    t1 = std::chrono::steady_clock::now();
    matcher->match(description_1,description_2,matches);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    fmt::print(fg(fmt::color::blue),"Match ORB cost time is {} second\n", time_used.count());
    
    // -- 4. Filterting the feature points
    // -- Computing the max and the min distance
    auto min_max = std::minmax_element(matches.begin(),matches.end(),
                    [](const DMatch& m1, const DMatch& m2){
                        return m1.distance < m2.distance;
                    });

    double min_diet = min_max.first->distance;
    double max_diet = min_max.second->distance;

    fmt::print(fg(fmt::color::green),"The min dist is  {}\n The max dis is {}\n",min_diet,max_diet);

    // -- 5. Judging: When the distance is twice greater than the minimum distance, then error
    std::vector<DMatch> good_matches;
    for(int i = 0; i < description_1.rows; ++i){
        if(matches[i].distance <= max(2 * min_diet,30.0))
            good_matches.emplace_back(matches[i]);
    }

    // -- 6. Matching the last result
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1,keypoint_1,img_2,keypoint_2,matches,img_match);
    drawMatches(img_1,keypoint_1,img_2,keypoint_2,good_matches,img_goodmatch);
    
    imshow("All mathes",img_match);
    imshow("Good matches",img_goodmatch);

    waitKey(0);

    return 0;
}