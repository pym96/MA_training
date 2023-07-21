#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisherNode : public rclcpp::Node
{
public:
  ImagePublisherNode() : Node("image_publisher_node")
  {
    // Subscribe to the image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera_image", 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          // Convert sensor_msgs::Image to cv::Mat using cv_bridge
          cv_bridge::CvImagePtr cv_ptr;
          try
          {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          }
          catch (cv_bridge::Exception& e)
          {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
          }

          // Process the image (you can perform any image processing here)
          cv::Mat processed_image = cv_ptr->image;

          // Publish the processed image to the "processed_image" topic
          cv_bridge::CvImage processed_msg;
          processed_msg.header = msg->header;
          processed_msg.encoding = sensor_msgs::image_encodings::BGR8;
          processed_msg.image = processed_image;

          processed_publisher_->publish(processed_msg.toImageMsg());
        });

    // Publish the processed image to the "processed_image" topic
    processed_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
