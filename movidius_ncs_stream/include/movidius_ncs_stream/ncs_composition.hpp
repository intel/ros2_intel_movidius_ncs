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

#ifndef MOVIDIUS_NCS_STREAM__NCS_COMPOSITION_HPP_
#define MOVIDIUS_NCS_STREAM__NCS_COMPOSITION_HPP_

#include <object_msgs/msg/objects.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <vector>

#include "movidius_ncs_lib/ncs_manager.hpp"
#include "movidius_ncs_lib/param.hpp"

namespace movidius_ncs_stream
{
typedef boost::function<void (movidius_ncs_lib::ClassificationResultPtr result,
    std_msgs::msg::Header header)>
  FUNP_C;
typedef boost::function<void (movidius_ncs_lib::DetectionResultPtr result,
    std_msgs::msg::Header header)>
  FUNP_D;

class NCSComposition : public rclcpp::Node
{
public:
  __attribute__((visibility("default"))) NCSComposition();
  ~NCSComposition();

  void cbGetClassificationResult(
    movidius_ncs_lib::ClassificationResultPtr result,
    std_msgs::msg::Header header);
  void cbGetDetectionResult(
    movidius_ncs_lib::DetectionResultPtr result,
    std_msgs::msg::Header header);

private:
  int encoding2mat_type(const std::string & encoding);
  void cbClassify(const sensor_msgs::msg::Image::SharedPtr img);
  void cbDetect(const sensor_msgs::msg::Image::SharedPtr img);
  void init();

  std::shared_ptr<movidius_ncs_lib::NCSManager> ncs_manager_handle_;

  rclcpp::Publisher<object_msgs::msg::Objects>::SharedPtr
    pub_classified_objects_;
  rclcpp::Publisher<object_msgs::msg::ObjectsInBoxes>::SharedPtr
    pub_detected_objects_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  movidius_ncs_lib::Param::Ptr param_;
  std::string input_topic_;
};
}  // namespace movidius_ncs_stream

#endif  // MOVIDIUS_NCS_STREAM__NCS_COMPOSITION_HPP_
