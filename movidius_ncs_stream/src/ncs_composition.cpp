/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "ament_index_cpp/get_resource.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include "movidius_ncs_lib/exception.hpp"
#include "movidius_ncs_lib/exception_util.hpp"
#include <object_msgs/srv/classify_object.hpp>
#include <object_msgs/srv/detect_object.hpp>
#include "movidius_ncs_stream/ncs_composition.hpp"

using movidius_ncs_lib::ClassificationResultPtr;
using movidius_ncs_lib::DetectionResultPtr;
using movidius_ncs_lib::Device;

namespace movidius_ncs_stream
{
template <typename T>
void operator>>(const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

NCSComposition::NCSComposition()
  : Node("movidius_ncs_stream"), ncs_handle_(nullptr), param_(nullptr)
{
  try
  {
    std::string content;
    std::string prefix_path;
    std::string line;
    std::string param_file;
    ament_index_cpp::get_resource("packages", "movidius_ncs_launch", content, &prefix_path);

    std::ifstream fin(prefix_path + "/share/movidius_ncs_launch/config/default.yaml");
    if (fin.fail())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not open default.yaml");
      rclcpp::shutdown();
    }

    YAML::Node doc = YAML::Load(fin);

    doc["param_file"] >> param_file;
    doc["input_topic"] >> input_topic_;

    if (param_file.empty() || input_topic_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "param_file or input_topic not set, please check "
                                       "default.yaml");
      rclcpp::shutdown();
    }
    else
    {
      param_ = std::make_shared<movidius_ncs_lib::Param>();
      param_file.erase(param_file.begin(),
                       std::find_if(param_file.begin(), param_file.end(),
                                    std::not1(std::ptr_fun<int, int>(std::isspace))));
      if (param_->loadParamFromYAML(prefix_path + "/share/movidius_ncs_launch/config/" +
                                    param_file) &&
          param_->validateParam())
      {
        init();
      }
      else
      {
        rclcpp::shutdown();
      }
    }
  }
  catch (YAML::InvalidScalar)
  {
    RCLCPP_ERROR(this->get_logger(), "The YAML file does not contain param_file & input_topic or it is "
                                     "invalid.");
    rclcpp::shutdown();
  }
  catch (movidius_ncs_lib::MvncException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    rclcpp::shutdown();
  }
  catch (...)
  {
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
  std::vector<float> mean = { param_->channel1_mean_, param_->channel2_mean_,
                              param_->channel3_mean_ };
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
    //qos.depth = 10;
    //qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    //qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    pub_classified_objects_ = create_publisher<object_msgs::msg::Objects>(
        "/movidius_ncs_stream/classified_objects", qos);
  }
  else
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_,
        std::bind(&NCSComposition::cbDetect, this, std::placeholders::_1),
        rmw_qos_profile_sensor_data);

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    //qos.depth = 10;
    //qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    //qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    pub_detected_objects_ = create_publisher<object_msgs::msg::ObjectsInBoxes>(
        "/movidius_ncs_stream/detected_objects", qos);
  }
}

int NCSComposition::encoding2mat_type(const std::string& encoding)
{
  if (encoding == "mono8")
  {
    return CV_8UC1;
  }
  else if (encoding == "bgr8")
  {
    return CV_8UC3;
  }
  else if (encoding == "mono16")
  {
    return CV_16SC1;
  }
  else if (encoding == "rgba8")
  {
    return CV_8UC4;
  }
  else if (encoding == "bgra8")
  {
    return CV_8UC4;
  }
  else if (encoding == "32FC1")
  {
    return CV_32FC1;
  }
  else if (encoding == "rgb8")
  {
    return CV_8UC3;
  }
  else
  {
    throw std::runtime_error("Unsupported encoding type");
  }
}

void NCSComposition::cbClassify(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  // cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  cv::Mat frame(image_msg->height, image_msg->width, encoding2mat_type(image_msg->encoding),
                const_cast<unsigned char*>(image_msg->data.data()), image_msg->step);
  cv::Mat cameraData;
  if (image_msg->encoding == "rgb8")
  {
    cv::cvtColor(frame, cameraData, cv::COLOR_RGB2BGR);
  }
  else
  {
    cameraData = frame;
  }
  // std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  ncs_handle_->loadTensor(cameraData);
  ncs_handle_->classify();
  ClassificationResultPtr result = ncs_handle_->getClassificationResult();
  // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  auto objs = std::make_shared<object_msgs::msg::Objects>();

  for (auto item : result->items)
  {
    object_msgs::msg::Object obj;
    obj.object_name = item.category;
    obj.probability = item.probability;
    objs->objects_vector.push_back(obj);
  }

  objs->header = image_msg->header;
  // Add origin image for example
  objs->image.height = image_msg->height;
  objs->image.width = image_msg->width;
  objs->image.encoding = image_msg->encoding;
  objs->image.step = image_msg->step;
  objs->image.data = image_msg->data;

  objs->inference_time_ms = result->time_taken;
  // objs->fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  // RCLCPP_INFO(this->get_logger(), "Total time: %fms", 1000.0 / objs->fps);
  // RCLCPP_INFO(this->get_logger(), "Inference time: %fms", objs_in_boxes->inference_time_ms);
  pub_classified_objects_->publish(objs);
}

void NCSComposition::cbDetect(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  // cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  cv::Mat frame(image_msg->height, image_msg->width, encoding2mat_type(image_msg->encoding),
                const_cast<unsigned char*>(image_msg->data.data()), image_msg->step);
  cv::Mat cameraData;
  if (image_msg->encoding == "rgb8")
  {
    cv::cvtColor(frame, cameraData, cv::COLOR_RGB2BGR);
  }
  else
  {
    cameraData = frame;
  }
  // std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  ncs_handle_->loadTensor(cameraData);
  ncs_handle_->detect();
  DetectionResultPtr result = ncs_handle_->getDetectionResult();
  // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  auto objs_in_boxes = std::make_shared<object_msgs::msg::ObjectsInBoxes>();

  for (auto item : result->items_in_boxes)
  {
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
  // Add origin image for example
  objs_in_boxes->image.height = image_msg->height;
  objs_in_boxes->image.width = image_msg->width;
  objs_in_boxes->image.encoding = image_msg->encoding;
  objs_in_boxes->image.step = image_msg->step;
  objs_in_boxes->image.data = image_msg->data;

  objs_in_boxes->inference_time_ms = result->time_taken;
  // objs_in_boxes->fps =
  //    1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  // RCLCPP_INFO(this->get_logger(), "Total time: %fms", 1000.0 / objs_in_boxes->fps);
  // RCLCPP_INFO(this->get_logger(), "Inference time: %fms", objs_in_boxes->inference_time_ms);
  pub_detected_objects_->publish(objs_in_boxes);
}
}  // namespace movidius_ncs_stream

#include <class_loader/register_macro.h>

CLASS_LOADER_REGISTER_CLASS(movidius_ncs_stream::NCSComposition, rclcpp::Node);
