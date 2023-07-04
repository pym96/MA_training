#include "training1_for_ma/transform.hpp"



int main(int argc, char** argv){
    
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<EigenDemoPublisher>());
  


    return 0;
}