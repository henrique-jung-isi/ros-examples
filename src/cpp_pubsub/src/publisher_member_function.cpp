// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#define IMAGE "/home/jung/Documents/ros-examples/test_image.png"
#define INTERVAL 200ms

using namespace std::chrono_literals;

rclcpp::NodeOptions options;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher", options), _count(0) {
    _image = cv::imread(IMAGE);
    _msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", _image).toImageMsg();
    _publisher = this->create_publisher<sensor_msgs::msg::Image>("pubExemple", 10);
    _timer = this->create_wall_timer(INTERVAL, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    _count++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", _count);
    // auto loaned = _publisher->borrow_loaned_message();
    // auto &msg = loaned.get();
    // msg.data = _msg->data;
    // msg.encoding = _msg->encoding;
    // msg.header = _msg->header;
    // msg.height = _msg->height;
    _msg->header.stamp = this->now();
    _publisher->publish(*_msg);
  }
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _publisher;
  cv::Mat _image;
  sensor_msgs::msg::Image::SharedPtr _msg;
  size_t _count;
};

int main(int argc, char *argv[]) {
  //   options.use_intra_process_comms(true);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
