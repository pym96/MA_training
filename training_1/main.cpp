#include "modules/cv/cv.hpp"
#include "modules/fmt_lib/fmt_lib.hpp"
#include<unordered_map>
#include<string>

int main(){

    cv::RNG rng;
    
    std::unordered_map<int,std::string> hash{{1,"ss"}};

    fmt::print(fg(fmt::color::red),"{}",hash[1]);

    // while(true){

    //     cv::Mat img(480,640,CV_8UC1,cv::Scalar(128));
    //     MA::CV cv;
    //     // fmt::print(fg(fmt::color::red)," Height {} \t Width {} \n",cv.getHeight(),cv.getWidth());

    //     int sum = 0;

    //     for(int i = 0; i < cv.getPointsAmount(); ++i){
    //         int x = rng.uniform(0,img.cols);
    //         int y = rng.uniform(0,img.rows);
            
    //         if(x >= (img.cols - 200) / 2 
    //            && x <= (img.cols - 200) / 2 + cv.getWidth() 
    //            && y >= ((img.rows - 100)) / 2
    //            && y <= (img.rows - 100) + cv.getHeight())

    //            sum++;

    //         cv::circle(img, cv::Point(x,y), 2, cv::Scalar(255));
    //     }

    //     cv::Rect rect((img.cols - 200) / 2, (img.rows - 100) / 2, cv.getWidth(), cv.getHeight());
    
    //     cv::rectangle(img,rect,cv::Scalar(0),2);
    
    //     cv::imshow("image",img);

    //     int key = cv::waitKey(10);

    //     fmt::print(fg(fmt::color::yellow),"Amount of points inside rect are {}\n",sum);
        
    //     if(key == 27) // Press Esc to break
    //          break;

    // }

    return 0;
}