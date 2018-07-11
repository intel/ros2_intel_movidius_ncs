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

#include "movidius_ncs_stream/ncs_composition.hpp"

#include <ament_index_cpp/get_resource.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <object_msgs/srv/classify_object.hpp>
#include <object_msgs/srv/detect_object.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <class_loader/register_macro.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "movidius_ncs_lib/exception.hpp"
#include "movidius_ncs_lib/exception_util.hpp"


using movidius_ncs_lib::ClassificationResultPtr;
using movidius_ncs_lib::DetectionResultPtr;
using movidius_ncs_lib::Device;

namespace movidius_ncs_stream
{
template<typename T>
void operator>>(const YAML::Node & node, T & i)
{
  i = node.as<T>();
}

NCSComposition::NCSComposition()
: Node("movidius_ncs_stream"), ncs_handle_(nullptr), param_(nullptr)
{
  try {
    std::string content;
    std::string prefix_path;
    std::string line;
    std::string param_file;
    ament_index_cpp::get_resource("packages", "movidius_ncs_launch", content, &prefix_path);

    std::ifstream fin(prefix_path + "/share/movidius_ncs_launch/config/default.yaml");
    if (fin.fail()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open default.yaml");
      rclcpp::shutdown();
    }

    YAML::Node doc = YAML::Load(fin);

    doc["param_file"] >> param_file;
    doc["input_topic"] >> input_topic_;

    if (param_file.empty() || input_topic_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "param_file or input_topic not set, please check "
        "default.yaml");
      rclcpp::shutdown();
    } else {
      param_ = std::make_shared<movidius_ncs_lib::Param>();
      param_file.erase(param_file.begin(),
        std::find_if(param_file.begin(), param_file.end(),
        std::not1(std::ptr_fun<int, int>(std::isspace))));
      if (param_->loadParamFromYAML(prefix_path + "/share/movidius_ncs_launch/config/" +
        param_file) && param_->validateParam())
      {
        init();
      } else {
        rclcpp::shutdown();
      }
    }
  } catch (YAML::InvalidScalar) {
    RCLCPP_ERROR(this->get_logger(),
      "The YAML file does not contain param_file & input_topic or it is " "invalid.");
    rclcpp::shutdown();
  } catch (movidius_ncs_lib::MvncException & e) {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    rclcpp::shutdown();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "exception caught while starting NCSNodelet");
    rclcpp::shutdown();
  }
}

NCSComposition::~NCSComposition()
{
}

void NCSComposition::init()
{
  RCLCPP_DEBUG(this->get_logger(), "NCSNodelet onInit");
  std::vector<float> mean = {param_->channel1_mean_, param_->channel2_mean_,
    param_->channel3_mean_};
  ncs_handle_ = std::make_shared<movidius_ncs_lib::NCS>(
    param_->device_index_, static_cast<Device::LogLevel>(param_->log_level_), param_->cnn_type_,
    param_->graph_file_path_, param_->category_file_path_, param_->network_dimension_, mean,
    param_->scale_, param_->top_n_);

  if (!param_->cnn_type_.compare("alexnet") || !param_->cnn_type_.compare("googlenet") ||
    !param_->cnn_type_.compare("inception_v1") || !param_->cnn_type_.compare("inception_v2") ||
    !param_->cnn_type_.compare("inception_v3") || !param_->cnn_type_.compare("inception_v4") ||
    !param_->cnn_type_.compare("mobilenet") || !param_->cnn_type_.compare("squeezenet"))
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      std::bind(&NCSComposition::cbClassify, this, std::placeholders::_1),
      rmw_qos_profile_sensor_data);

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    // qos.depth = 10;
    // qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    // qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    pub_classified_objects_ = create_publisher<object_msgs::msg::Objects>(
      "/movidius_ncs_stream/classified_objects", qos);
  } else {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      std::bind(&NCSComposition::cbDetect, this, std::placeholders::_1),
      rmw_qos_profile_sensor_data);

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    // qos.depth = 10;
    // qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    // qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    pub_detected_objects_ = create_publisher<object_msgs::msg::ObjectsInBoxes>(
      "/movidius_ncs_stream/detected_objects", qos);
  }
}

void NCSComposition::cbClassify(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  cv::Mat cameraData = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  ncs_handle_->loadTensor(cameraData);
  ncs_handle_->classify();
  ClassificationResultPtr result = ncs_handle_->getClassificationResult();
  auto objs = std::make_shared<object_msgs::msg::Objects>();

  for (auto item : result->items) {
    object_msgs::msg::Object obj;
    obj.object_name = item.category;
    obj.probability = item.probability;
    objs->objects_vector.push_back(obj);
  }

  objs->header = image_msg->header;
  objs->inference_time_ms = result->time_taken;
  pub_classified_objects_->publish(objs);
}

void NCSComposition::cbDetect(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  cv::Mat cameraData = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  ncs_handle_->loadTensor(cameraData);
  ncs_handle_->detect();
  DetectionResultPtr result = ncs_handle_->getDetectionResult();
  auto objs_in_boxes = std::make_shared<object_msgs::msg::ObjectsInBoxes>();

  for (auto item : result->items_in_boxes) {
    object_msgs::msg::ObjectInBox obj;
    obj.object.object_name = item.item.category;
    obj.object.probability = item.item.probability;
    obj.roi.x_offset = item.bbox.x;
    obj.roi.y_offset = item.bbox.y;
    obj.roi.width = item.bbox.width;
    obj.roi.height = item.bbox.height;
    objs_in_boxes->objects_vector.push_back(obj);
  }

  objs_in_boxes->header = image_msg->header;
  objs_in_boxes->inference_time_ms = result->time_taken;
  pub_detected_objects_->publish(objs_in_boxes);
}
}  // namespace movidius_ncs_stream

CLASS_LOADER_REGISTER_CLASS(movidius_ncs_stream::NCSComposition, rclcpp::Node);
