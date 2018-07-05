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

#ifndef MOVIDIUS_NCS_IMAGE__NCS_SERVER_HPP_
#define MOVIDIUS_NCS_IMAGE__NCS_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <object_msgs/srv/classify_object.hpp>
#include <object_msgs/srv/detect_object.hpp>

#include <memory>
#include <string>
#include <vector>

#include "movidius_ncs_lib/ncs.hpp"
#include "movidius_ncs_lib/param.hpp"

namespace movidius_ncs_image
{
class NCSServer : public rclcpp::Node
{
public:
  explicit NCSServer(const std::string & service_name);

private:
  void getParameters();
  void init(const std::string & service_name);

  void cbClassifyObject(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<object_msgs::srv::ClassifyObject::Request> request,
    std::shared_ptr<object_msgs::srv::ClassifyObject::Response> response);
  void cbDetectObject(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<object_msgs::srv::DetectObject::Request> request,
    std::shared_ptr<object_msgs::srv::DetectObject::Response> response);

  rclcpp::Service<object_msgs::srv::ClassifyObject>::SharedPtr service_classify_;
  rclcpp::Service<object_msgs::srv::DetectObject>::SharedPtr service_detect_;

  std::shared_ptr<movidius_ncs_lib::NCS> ncs_handle_;

  movidius_ncs_lib::Param::Ptr param_;
};
}  // namespace movidius_ncs_image

#endif  // MOVIDIUS_NCS_IMAGE__NCS_SERVER_HPP_
