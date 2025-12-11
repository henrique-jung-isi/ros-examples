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

using std::placeholders::_1;

rclcpp::NodeOptions options;
class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber", options) {
    _subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "pubExemple", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image &msg) const {
    rclcpp::Time stamp(msg.header.stamp, this->get_clock()->get_clock_type());
    auto age = now() - stamp;
    RCLCPP_INFO(this->get_logger(), "Image age: %.3f ms", age.seconds() * 1e3);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
};

int main(int argc, char *argv[]) {
  //   options.use_intra_process_comms(true);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
