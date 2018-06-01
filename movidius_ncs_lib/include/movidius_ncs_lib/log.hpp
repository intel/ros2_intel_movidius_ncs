// Copyright (c) 2017 Intel Corporation. All Rights Reserved
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

#ifndef MOVIDIUS_NCS_LIB__LOG_HPP_
#define MOVIDIUS_NCS_LIB__LOG_HPP_

#include <iostream>

namespace movidius_ncs_lib
{
#define ROS_ERROR_STREAM(str) std::cout << "[ERROR] [movidius_ncs_lib]: " << str << std::endl

#define ROS_WARN_STREAM(str) std::cout << "[WARN] [movidius_ncs_lib]: " << str << std::endl

#define ROS_INFO_STREAM(str) std::cout << "[INFO] [movidius_ncs_lib]: " << str << std::endl

#define ROS_ERROR(str) std::cout << "[ERROR] [movidius_ncs_lib]: " << str << std::endl

#define ROS_WARN(str) std::cout << "[WARN] [movidius_ncs_lib]: " << str << std::endl

#define ROS_INFO(str) std::cout << "[INFO] [movidius_ncs_lib]: " << str << std::endl

#define ROS_DEBUG(str) std::cout << "[DEBUG] [movidius_ncs_lib]: " << str << std::endl
}  // namespace movidius_ncs_lib
#endif  // MOVIDIUS_NCS_LIB__LOG_HPP_
