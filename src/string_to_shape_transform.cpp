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
#include <sstream>
#include <string>

#include "test_network_type_adapters/string_to_shape_transform.hpp"

#include "std_msgs/msg/string.hpp"

extern "C" {
  #include "ShapeTypeSupport.h"
}

#ifdef _WIN32
// This is necessary because of a bug in yaml-cpp's cmake
#define YAML_CPP_DLL
// This is necessary because yaml-cpp does not always use dllimport/dllexport consistently
# pragma warning(push)
# pragma warning(disable:4251)
# pragma warning(disable:4275)
#endif
#include "yaml-cpp/yaml.h"
#ifdef _WIN32
# pragma warning(pop)
#endif

RMW_CONNEXTDDS_C_TYPE_SUPPORT_ADAPTER(ShapeType);

template<>
struct rmw_connextdds::TypeTransformation<std_msgs::msg::String, ShapeType>
{
  static void application_to_network(
    const std_msgs::msg::String & application_data,
    ShapeType & network_data)
  {
    YAML::Node yaml_shape = YAML::Load(application_data.data);
    network_data.color = DDS_String_dup(
      yaml_shape["color"].as<std::string>().c_str());
    network_data.x = yaml_shape["x"].as<int>();
    network_data.y = yaml_shape["y"].as<int>();
    network_data.shapesize = yaml_shape["shapesize"].as<int>();
  }

  static void network_to_application(
    const ShapeType & network_data,
    std_msgs::msg::String & application_data)
  {
    std::stringstream ss;
    ss << "{ \"color\": \"" << network_data.color << "\", "
      << "\"x\": " << network_data.x << ", "
      << "\"y\": " << network_data.y << ", "
      << "\"shapesize\": " << network_data.shapesize << " }";
    application_data.data = ss.str();
  }
};

rmw_connextdds::AnyTypeTransformationPlugin*
StringToShapeTransformationPlugin_create(
  DDS_DomainParticipant * const participant,
  const char * const application_type_name,
  const char * const network_type_name)
{
  return new (std::nothrow) rmw_connextdds::TypeTransformationPlugin<
    std_msgs::msg::String, ShapeType>(
      participant, application_type_name, network_type_name);
}
