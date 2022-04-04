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
#include "test_network_type_adapters/image_transform_c.hpp"

#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#include "sensor_msgs/msg/image.h"

extern "C" {
  #include "ros2/sensor_msgs/msg/ImageSupport.h"
}

RMW_CONNEXTDDS_ZERO_COPY_C_TYPE_SUPPORT_ADAPTER(ros2_sensor_msgs_msg_Image);

template<>
struct rmw_connextdds::TypeTransformation<sensor_msgs__msg__Image, ros2_sensor_msgs_msg_Image>
{
  static void application_to_network(
    const sensor_msgs__msg__Image & app_data,
    ros2_sensor_msgs_msg_Image & net_data)
  {
    net_data.header.stamp.sec = app_data.header.stamp.sec;
    net_data.header.stamp.nanosec = app_data.header.stamp.nanosec;
    const auto frame_id_len = app_data.header.frame_id.size;
    if (frame_id_len > 0) {
      if (frame_id_len > ros2_std_msgs_msg_FRAME_ID_MAX_LENGTH) {
        throw std::runtime_error("frame_id is too long");
      }
      memcpy(
        net_data.header.frame_id,
        app_data.header.frame_id.data,
        frame_id_len + 1);
    }

    net_data.height = app_data.height;
    net_data.width = app_data.width;
    net_data.step = app_data.step;
    const auto encoding_len = app_data.encoding.size;
    if (encoding_len > 0) {
      if (encoding_len > ros2_sensor_msgs_msg_ENCODING_MAX_LENGTH) {
        throw std::runtime_error("encoding is too long");
      }
      memcpy(
        net_data.encoding,
        app_data.encoding.data,
        encoding_len + 1);
    }

    auto data_size = app_data.data.size;
    if (data_size > ros2_sensor_msgs_msg_IMAGE_MAX_SIZE) {
      throw std::runtime_error("data_size is too large");
    }
    memcpy(&net_data.data, app_data.data.data, data_size);
    net_data.data_len = data_size;
  }

  static void network_to_application(
    const ros2_sensor_msgs_msg_Image & net_data,
    sensor_msgs__msg__Image & app_data)
  {
    app_data.header.stamp.sec = net_data.header.stamp.sec;
    app_data.header.stamp.nanosec = net_data.header.stamp.nanosec;
    if (!rosidl_runtime_c__String__assign(
      &app_data.header.frame_id,
      reinterpret_cast<const char *>(&net_data.header.frame_id)))
    {
      throw std::runtime_error("failed to assign frame_id");
    }

    app_data.height = net_data.height;
    app_data.width = net_data.width;
    app_data.step = net_data.step;
    if (!rosidl_runtime_c__String__assign(
      &app_data.encoding,
      reinterpret_cast<const char *>(&net_data.encoding)))
    {
      throw std::runtime_error("failed to assign encoding");
    }

    if (app_data.data.capacity < net_data.data_len) {
      rosidl_runtime_c__uint8__Sequence__fini(&app_data.data);
      if (!rosidl_runtime_c__uint8__Sequence__init(&app_data.data, net_data.data_len)) {
        throw std::runtime_error("failed to finalize data sequence");
      }
    }
    memcpy(app_data.data.data, &net_data.data, net_data.data_len);
    app_data.data.size = net_data.data_len;
  }
};

rmw_connextdds::AnyTypeTransformationPlugin*
ImageCTransformationPlugin_create(
  DDS_DomainParticipant * const participant,
  const char * const application_type_name,
  const char * const network_type_name)
{
  return new (std::nothrow) rmw_connextdds::TypeTransformationPlugin<
    sensor_msgs__msg__Image, ros2_sensor_msgs_msg_Image>(
      participant, application_type_name, network_type_name);
}