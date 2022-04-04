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
#ifndef TEST_NETWORK_TYPE_ADAPTERS__IMAGE_ADAPTER_HPP_
#define TEST_NETWORK_TYPE_ADAPTERS__IMAGE_ADAPTER_HPP_

#include "test_network_type_adapters/visibility_control.h"

#include "rmw_connextdds/type_transformation.hpp"

extern "C" {
TEST_NETWORK_TYPE_ADAPTERS_PUBLIC
rmw_connextdds::AnyTypeTransformationPlugin*
ImageCTransformationPlugin_create(
  DDS_DomainParticipant * const participant,
  const char * const application_type_name,
  const char * const network_type_name);
}  // extern "C"

#endif  // TEST_NETWORK_TYPE_ADAPTERS__IMAGE_ADAPTER_HPP_
