#ifndef __MA_SUBSRIBER_HPP__
#define __MA_SUBSRIBER_HPP__

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <functional>

class MASubsriber: public rclcpp::Node
{
    public:
        MASubsriber()
        :Node("demo_subsriber"){
            subscription_ = this->create_subscription<std_msgs::msg::String>("MA_train",rclcpp::QoS(rclcpp::KeepAll()).best_effort(), std::bind(&MASubsriber::sub_callback,this,std::placeholders::_1));
        }

    private:
        void sub_callback(const std_msgs::msg::String& msg) const{
            RCLCPP_INFO(this->get_logger(),"Current info is %s",msg.data.c_str());
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


};

#endif