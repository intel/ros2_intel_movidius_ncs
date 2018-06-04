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


#include "movidius_ncs_lib/param.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>

#include "movidius_ncs_lib/device.hpp"
#include "movidius_ncs_lib/log.hpp"

namespace movidius_ncs_lib
{
template<typename T>
void operator>>(const YAML::Node & node, T & i)
{
  i = node.as<T>();
}

Param::Param()
: device_index_(0),
  log_level_(Device::Errors), cnn_type_(""),
  graph_file_path_(""),
  category_file_path_(""),
  network_dimension_(0),
  channel1_mean_(0),
  channel2_mean_(0),
  channel3_mean_(0),
  scale_(1.0),
  top_n_(1)
{
}

bool Param::loadParamFromYAML(const std::string & file_path)
{
  std::ifstream fin(file_path);
  if (fin.fail()) {
    ROS_ERROR_STREAM("Could not open " << file_path);
    return false;
  }

  YAML::Node doc = YAML::Load(fin);

  try {
    doc["device_index"] >> device_index_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a device_index tag or it is invalid.");
    return false;
  }

  try {
    doc["log_level"] >> log_level_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a log_level tag or it is invalid.");
    return false;
  }

  try {
    doc["top_n"] >> top_n_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a top_n tag or it is invalid.");
    return false;
  }

  try {
    doc["cnn_type"] >> cnn_type_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a cnn_type tag or it is invalid.");
    return false;
  }

  try {
    doc["graph_file_path"] >> graph_file_path_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a device_index tag or it is invalid.");
    return false;
  }

  try {
    doc["category_file_path"] >> category_file_path_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a category_file_path tag or it is invalid.");
    return false;
  }

  try {
    doc["network_dimension"] >> network_dimension_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a device_index tag or it is invalid.");
    return false;
  }

  try {
    doc["channel1_mean"] >> channel1_mean_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a channel1_mean tag or it is invalid.");
    return false;
  }

  try {
    doc["channel2_mean"] >> channel2_mean_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a channel2_mean tag or it is invalid.");
    return false;
  }

  try {
    doc["channel3_mean"] >> channel3_mean_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a channel3_mean tag or it is invalid.");
    return false;
  }

  try {
    doc["scale"] >> scale_;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("The YAML file does not contain a scale tag or it is invalid.");
    return false;
  }
  return true;
}

bool Param::validateParam()
{
  ROS_DEBUG("NCSImpl get parameters");

  if (device_index_ < 0) {
    ROS_ERROR_STREAM("invalid param device_index = " << device_index_);
    throw std::exception();
    return false;
  }

  ROS_INFO_STREAM("use device_index = " << device_index_);

  if (log_level_ < Device::Nothing || log_level_ > Device::Verbose) {
    ROS_WARN_STREAM("invalid param log_level = " << log_level_);
    throw std::exception();
    return false;
  }

  ROS_INFO_STREAM("use log_level = " << log_level_);

  if (!std::fstream(graph_file_path_)) {
    ROS_ERROR_STREAM("graph_file_path = " << graph_file_path_ << " not exists");
    throw std::exception();
    return false;
  }

  ROS_INFO_STREAM("use graph_file_path = " << graph_file_path_);

  if (!std::fstream(category_file_path_)) {
    ROS_ERROR_STREAM("category_file_path = " << category_file_path_ << " not exists");
    throw std::exception();
    return false;
  }

  ROS_INFO_STREAM("use category_file_path = " << category_file_path_);

  if (cnn_type_.compare("alexnet") && cnn_type_.compare("googlenet") &&
    cnn_type_.compare("inception_v1") && cnn_type_.compare("inception_v2") &&
    cnn_type_.compare("inception_v3") && cnn_type_.compare("inception_v4") &&
    cnn_type_.compare("mobilenet") && cnn_type_.compare("squeezenet") &&
    cnn_type_.compare("tinyyolo_v1") && cnn_type_.compare("mobilenetssd"))
  {
    ROS_WARN_STREAM("invalid cnn_type_=" << cnn_type_);
    throw std::exception();
    return false;
  }

  ROS_INFO_STREAM("use cnn_type_ = " << cnn_type_);

  if (network_dimension_ < 0) {
    ROS_WARN_STREAM("invalid network_dimension = " << network_dimension_);
    throw std::exception();
    return false;
  }

  ROS_INFO_STREAM("use network_dimension = " << network_dimension_);

  ROS_INFO_STREAM("use channel1_mean = " << channel1_mean_);
  ROS_INFO_STREAM("use channel2_mean = " << channel2_mean_);
  ROS_INFO_STREAM("use channel3_mean = " << channel3_mean_);

  if (top_n_ < 1) {
    ROS_WARN_STREAM("invalid top_n = " << top_n_);
    throw std::exception();
    return false;
  }

  ROS_INFO_STREAM("use top_n = " << top_n_);

  if (scale_ < 0) {
    ROS_WARN_STREAM("invalid param scale = " << scale_);
    throw std::exception();
    return false;
  }

  ROS_INFO_STREAM("use scale = " << scale_);
  return true;
}
}  // namespace movidius_ncs_lib
