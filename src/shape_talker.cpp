// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "test_network_type_adapters/visibility_control.h"

using namespace std::chrono_literals;

namespace test_network_type_adapters
{
class ShapeTalker : public rclcpp::Node
{
public:
  TEST_NETWORK_TYPE_ADAPTERS_PUBLIC
  explicit ShapeTalker(const rclcpp::NodeOptions & options)
  : Node("shape_talker", options)
  {
    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::String>();
        std::stringstream ss;
        ss << "{ \"color\": \"RED\", "
          << "\"x\": " << 125 << ", "
          << "\"y\": " << (count_++ % 255) << ", "
          << "\"shapesize\": 50 }";
        msg_->data = ss.str();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
        pub_->publish(std::move(msg_));
      };
    rclcpp::QoS qos(rclcpp::KeepLast{7});
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(50ms, publish_message);
  }

private:
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace test_network_type_adapters

RCLCPP_COMPONENTS_REGISTER_NODE(test_network_type_adapters::ShapeTalker)
