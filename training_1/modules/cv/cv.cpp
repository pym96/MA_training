
#include"cv.hpp"

namespace MA{

    CV::CV(){
        cv::FileStorage file(PROJECT_DIR "/config/detect/detect.yaml",cv::FileStorage::READ);
        file["settings"]["points_amount"] >> points_amount;
        file["settings"]["rec_width"] >> width;
        file["settings"]["rec_height"] >> height;
    }
    
}