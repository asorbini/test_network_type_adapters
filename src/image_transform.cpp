// Copyright 2022 Real-Time Innovations, Inc. (RTI)
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
#include "test_network_type_adapters/image_transform.hpp"

#include "sensor_msgs/msg/image.hpp"

extern "C" {
  #include "ros2/sensor_msgs/msg/ImageSupport.h"
}

RMW_CONNEXTDDS_ZERO_COPY_C_TYPE_SUPPORT_ADAPTER(ros2_sensor_msgs_msg_Image);

template<>
struct rmw_connextdds::TypeTransformation<sensor_msgs::msg::Image, ros2_sensor_msgs_msg_Image>
{
  static void application_to_network(
    const sensor_msgs::msg::Image & app_data,
    ros2_sensor_msgs_msg_Image & net_data)
  {
    net_data.header.stamp.sec = app_data.header.stamp.sec;
    net_data.header.stamp.nanosec = app_data.header.stamp.nanosec;
    const auto frame_id_len = app_data.header.frame_id.length();
    if (frame_id_len > 0) {
      if (frame_id_len > ros2_std_msgs_msg_FRAME_ID_MAX_LENGTH) {
        throw std::runtime_error("frame_id is too long");
      }
      memcpy(
        net_data.header.frame_id,
        app_data.header.frame_id.c_str(),
        frame_id_len + 1);
    }

    net_data.height = app_data.height;
    net_data.width = app_data.width;
    net_data.step = app_data.step;
    const auto encoding_len = app_data.encoding.length();
    if (encoding_len > 0) {
      if (encoding_len > ros2_sensor_msgs_msg_ENCODING_MAX_LENGTH) {
        throw std::runtime_error("encoding is too long");
      }
      memcpy(
        net_data.encoding,
        app_data.encoding.c_str(),
        encoding_len + 1);
    }

    auto data_size = app_data.data.size();
    if (data_size > ros2_sensor_msgs_msg_IMAGE_MAX_SIZE) {
      throw std::runtime_error("data_size is too large");
    }
    memcpy(&net_data.data, &app_data.data[0], data_size);
    net_data.data_len = data_size;
  }

  static void network_to_application(
    const ros2_sensor_msgs_msg_Image & net_data,
    sensor_msgs::msg::Image & app_data)
  {
    app_data.header.stamp.sec = net_data.header.stamp.sec;
    app_data.header.stamp.nanosec = net_data.header.stamp.nanosec;
    const char * const frame_id =
      reinterpret_cast<const char *>(&net_data.header.frame_id);
    app_data.header.frame_id = frame_id;

    app_data.height = net_data.height;
    app_data.width = net_data.width;
    app_data.step = net_data.step;
    const char * const encoding =
      reinterpret_cast<const char *>(&net_data.encoding);
    app_data.encoding = encoding;

    app_data.data.resize(net_data.data_len);
    memcpy(&app_data.data[0], &net_data.data, net_data.data_len);
  }
};

rmw_connextdds::AnyTypeTransformationPlugin*
ImageTransformationPlugin_create(
  DDS_DomainParticipant * const participant,
  const char * const application_type_name,
  const char * const network_type_name)
{
  return new (std::nothrow) rmw_connextdds::TypeTransformationPlugin<
    sensor_msgs::msg::Image, ros2_sensor_msgs_msg_Image>(
      participant, application_type_name, network_type_name);
}