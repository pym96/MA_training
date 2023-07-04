#ifndef __TRANSFORM_HPP__
#define __TRANSFORM_HPP__

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class EigenDemoPublisher: public rclcpp::Node
{
    public:
        EigenDemoPublisher()
        : Node("demo_publisher"), count_(0){
            publisher_ = this->create_publisher<std_msgs::msg::String>("MA_train", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());   
            timer_ = this->create_wall_timer(500ms,std::bind(&EigenDemoPublisher::publishCallback,this));
        }

    private:
        void publishCallback(){
            Eigen::Matrix<double, 2, 6> points;
            points << 2, 2, 0.5, -1, -1, 2,
                     -1, 2, 3, 2, -1, -1;
            
            Eigen::MatrixXd shear_mat(2,2);
            shear_mat << 1, 0.7,
                        0.2, 1;
            
            srand((unsigned)(time(nullptr)));
            int i = rand() % 10;
            
            // RCLCPP_INFO(this->get_logger(),"Getting inside of callback function\n");
            
            
            shear_mat += 0.1 * i * Eigen::MatrixXd::Random(2,2);

            auto result = shear_mat * points;
            
            auto message = std_msgs::msg::String();
            message.data = "----" + i;
            RCLCPP_INFO(this->get_logger(),"Getting inside of callback function value is %s\n",message.data.c_str());

            publisher_->publish(message);
            
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

     
};

#endif