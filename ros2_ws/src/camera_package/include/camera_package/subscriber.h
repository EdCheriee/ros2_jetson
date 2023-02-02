#include <memory>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <experimental/filesystem>
namespace efs = std::experimental::filesystem;


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image/pub", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      

    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      cv::Mat img;

      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
      const auto p1 = std::chrono::system_clock::now();
      cv::imwrite("/camera_test/CSI-Camera/images/" + std::to_string(p1.time_since_epoch().count()) + ".png", img);
    }


    std::string directory_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};
