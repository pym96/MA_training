#include "MA_susbriber/MA_subsriber.hpp"


int main(int argc, char** argv){

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MASubsriber>());
    

    return 0;
}