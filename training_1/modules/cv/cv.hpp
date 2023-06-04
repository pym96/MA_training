#ifndef __CV_HPP__
#define __CV_HPP__

#include<opencv2/opencv.hpp>

namespace MA{

    class CV{
        public:
            CV();
            ~CV() = default;
            CV(CV const& ) = delete;
            CV & operator=(CV const &) = delete;
            
        public:
            int getPointsAmount(){
                return this->points_amount;
            }

            int getWidth(){
                return this->width;
            }

            int getHeight(){
                return this->height;
            }
            

        private:
            int points_amount;
            int width;
            int height;
    };
    
}


#endif