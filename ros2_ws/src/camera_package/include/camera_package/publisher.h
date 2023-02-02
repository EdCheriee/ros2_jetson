#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      int capture_width = 1280 ;
      int capture_height = 720 ;
      int display_width = 1280 ;
      int display_height = 720 ;
      int framerate = 30 ;
      int flip_method = 0 ;

      std::string pipeline = gstreamer_pipeline(capture_width,
      capture_height,
      display_width,
      display_height,
      framerate,
      flip_method);

      std::cout << "Using pipeline: \n\t" << pipeline << "\n";

      cap_.open(pipeline, cv::CAP_GSTREAMER);

      if(!cap_.isOpened()) 
      {
          std::cout<<"Failed to open camera."<<std::endl;
          return;
      }

      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image/pub", 10);
      timer_ = this->create_wall_timer(
      5000ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
    ~MinimalPublisher() {cap_.release();}

  private:

    void timer_callback()
    {


        if (!cap_.read(img_)) 
        {
            std::cout<<"Capture read error"<<std::endl;
            return;
        }

        std::shared_ptr<sensor_msgs::msg::Image> msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_).toImageMsg();


        auto temp = msg.get();

        publisher_->publish(*temp);
    }

    std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) 
    {
      return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
            std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
            "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
            std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::Mat img_;
    size_t count_;
};
