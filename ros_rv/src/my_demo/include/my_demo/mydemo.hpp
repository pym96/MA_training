#ifndef __MY_DEMO_NODE__
#define __MY_DEMO_NODE__

#include <string>
#include "rclcpp/rclcpp.hpp"
#include <iostream>

namespace ros_demo{
    class Demo: public rclcpp::Node{
        public:
            explicit Demo(const std::string& node_name);
            ~Demo();

        private:
            rclcpp::TimerBase::SharedPtr printimer_;    
    };
}


#endif