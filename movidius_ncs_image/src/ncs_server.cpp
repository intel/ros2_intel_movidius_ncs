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

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "ament_index_cpp/get_resource.hpp"
#include "movidius_ncs_image/ncs_server.hpp"
#include "movidius_ncs_lib/exception.hpp"
#include "movidius_ncs_lib/exception_util.hpp"

using movidius_ncs_lib::ClassificationResultPtr;
using movidius_ncs_lib::DetectionResultPtr;
using movidius_ncs_lib::Device;

namespace movidius_ncs_image
{
NCSServer::NCSServer(const std::string & service_name)
: Node("movidius_ncs_image"),
  ncs_manager_handle_(nullptr),
  param_(nullptr)
{
  try {
    std::string content;
    std::string prefix_path;
    std::string line;
    std::string param_file;
    ament_index_cpp::get_resource("packages", "movidius_ncs_launch", content,
      &prefix_path);
    std::ifstream fin(prefix_path +
      "/share/movidius_ncs_launch/config/default.yaml");
    if (std::getline(fin, line)) {
      if (line.find("param_file") != std::string::npos) {
        param_file = line.substr(line.find(":") + 1);
      }
    }
    if (param_file.empty()) {
      RCLCPP_ERROR(this->get_logger(),
        "param_file not set, please check default.yaml");
      rclcpp::shutdown();
    } else {
      param_ = std::make_shared<movidius_ncs_lib::Param>();
      param_file.erase(
        param_file.begin(),
        std::find_if(param_file.begin(), param_file.end(),
        std::not1(std::ptr_fun<int, int>(std::isspace))));
      if (param_->loadParamFromYAML(prefix_path +
        "/share/movidius_ncs_launch/config/" +
        param_file) &&
        param_->validateParam())
      {
        init(service_name);
      } else {
        rclcpp::shutdown();
      }
    }
  } catch (movidius_ncs_lib::MvncException & e) {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    rclcpp::shutdown();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
      "exception caught while starting NCSServer");
    rclcpp::shutdown();
  }
}

void NCSServer::init(const std::string & service_name)
{
  RCLCPP_DEBUG(this->get_logger(), "NCSServer onInit");
  std::vector<float> mean = {param_->channel1_mean_, param_->channel2_mean_,
    param_->channel3_mean_};
  ncs_manager_handle_ = std::make_shared<movidius_ncs_lib::NCSManager>(
    param_->max_device_count_, param_->start_device_index_,
    static_cast<Device::LogLevel>(param_->log_level_), param_->cnn_type_,
    param_->graph_file_path_, param_->category_file_path_,
    param_->network_dimension_, mean, param_->scale_, param_->top_n_);

  if (!param_->cnn_type_.compare("alexnet") ||
    !param_->cnn_type_.compare("googlenet") ||
    !param_->cnn_type_.compare("inception_v1") ||
    !param_->cnn_type_.compare("inception_v2") ||
    !param_->cnn_type_.compare("inception_v3") ||
    !param_->cnn_type_.compare("inception_v4") ||
    !param_->cnn_type_.compare("mobilenet") ||
    !param_->cnn_type_.compare("squeezenet"))
  {
    service_classify_ = create_service<object_msgs::srv::ClassifyObject>(
      service_name + "/classify_object",
      std::bind(&NCSServer::cbClassifyObject, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));
  } else {
    service_detect_ = create_service<object_msgs::srv::DetectObject>(
      service_name + "/detect_object",
      std::bind(&NCSServer::cbDetectObject, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));
  }
}

void NCSServer::cbClassifyObject(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<object_msgs::srv::ClassifyObject::Request> request,
  std::shared_ptr<object_msgs::srv::ClassifyObject::Response> response)
{
  RCLCPP_WARN(this->get_logger(), "cbClassifyObject");

  std::vector<ClassificationResultPtr> results =
    ncs_manager_handle_->classifyImage(request->image_paths);

  for (unsigned int i = 0; i < results.size(); i++) {
    object_msgs::msg::Objects objs;
    for (auto item : results[i]->items) {
      object_msgs::msg::Object obj;
      obj.object_name = item.category;
      obj.probability = item.probability;
      objs.objects_vector.push_back(obj);
    }

    objs.inference_time_ms = results[i]->time_taken;
    response->objects.push_back(objs);
  }
}

void NCSServer::cbDetectObject(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<object_msgs::srv::DetectObject::Request> request,
  std::shared_ptr<object_msgs::srv::DetectObject::Response> response)
{
  RCLCPP_WARN(this->get_logger(), "cbDetectObject");
  std::vector<DetectionResultPtr> results =
    ncs_manager_handle_->detectImage(request->image_paths);

  for (unsigned int i = 0; i < results.size(); i++) {
    object_msgs::msg::ObjectsInBoxes objs;
    for (auto item : results[i]->items_in_boxes) {
      object_msgs::msg::ObjectInBox obj;
      obj.object.object_name = item.item.category;
      obj.object.probability = item.item.probability;
      obj.roi.x_offset = item.bbox.x;
      obj.roi.y_offset = item.bbox.y;
      obj.roi.width = item.bbox.width;
      obj.roi.height = item.bbox.height;
      objs.objects_vector.push_back(obj);
    }

    objs.inference_time_ms = results[i]->time_taken;
    response->objects.push_back(objs);
  }
}
}  // namespace movidius_ncs_image

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<movidius_ncs_image::NCSServer>("movidius_ncs_image");
    rclcpp::spin(node);
  } catch (...) {
    std::cout << "[ERROR] [movidius_ncs_image]: " <<
      "exception caught in movidius_ncs_node" << std::endl;
  }
}
