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

#include <object_msgs/srv/detect_object.hpp>
#include <object_msgs/srv/classify_object.hpp>

#include <string>
#include <fstream>
#include <chrono>
#include <memory>
#include <functional>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "movidius_ncs_lib/param.hpp"

bool cnn_type_flag = false;

void getCnnType()
{
  std::string content;
  std::string prefix_path;
  std::string line;
  std::string param_file;
  ament_index_cpp::get_resource("packages", "movidius_ncs_launch", content, &prefix_path);
  std::ifstream fin(prefix_path + "/share/movidius_ncs_launch/config/default.yaml");
  if (std::getline(fin, line)) {
    param_file = line.substr(line.find(":") + 1);
  }
  auto param_ = std::make_shared<movidius_ncs_lib::Param>();
  param_file.erase(param_file.begin(),
    std::find_if(param_file.begin(), param_file.end(),
    std::not1(std::ptr_fun<int, int>(std::isspace))));
  if (param_->loadParamFromYAML(prefix_path +
    "/share/movidius_ncs_launch/config/" +
    param_file))
  {
    if (!param_->cnn_type_.compare("alexnet") ||
      !param_->cnn_type_.compare("googlenet") ||
      !param_->cnn_type_.compare("inception_v1") ||
      !param_->cnn_type_.compare("inception_v2") ||
      !param_->cnn_type_.compare("inception_v3") ||
      !param_->cnn_type_.compare("inception_v4") ||
      !param_->cnn_type_.compare("mobilenet") ||
      !param_->cnn_type_.compare("squeezenet"))
    {
      cnn_type_flag = false;
    } else {
      cnn_type_flag = true;
    }
  }
}

std::string generate_file_path(std::string path)
{
  std::string base_path = __FILE__;
  const std::string filename = "movidius_ncs_image/tests/unittest_image.cpp";
  base_path = base_path.substr(0, base_path.length() - filename.length() - 1);
  return base_path + "/" + path;
}

TEST(UnitTestImage, testImage) {
  if (cnn_type_flag) {
    auto node = rclcpp::Node::make_shared("movidius_ncs_image_tests");

    auto client = node->create_client<object_msgs::srv::DetectObject>("movidius_ncs_image/"
        "detect_object");
    auto request = std::make_shared<object_msgs::srv::DetectObject::Request>();

    std::string buffer = generate_file_path("data/images/bicycle.jpg");
    request->image_path = buffer;

    if (!client->wait_for_service(std::chrono::seconds(20))) {
      ASSERT_TRUE(false) << "service not available after waiting";
    }

    auto result = client->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(5));
    EXPECT_EQ(ret, rclcpp::executor::FutureReturnCode::SUCCESS);

    auto srv = result.get();

    EXPECT_TRUE(srv->objects.objects_vector.size());
    EXPECT_EQ(srv->objects.objects_vector[0].object.object_name, "bicycle");
    EXPECT_TRUE(srv->objects.objects_vector[0].roi.x_offset > 360 &&
      srv->objects.objects_vector[0].roi.x_offset < 375 &&
      srv->objects.objects_vector[0].roi.y_offset > 265 &&
      srv->objects.objects_vector[0].roi.y_offset < 280 &&
      srv->objects.objects_vector[0].roi.width > 460 &&
      srv->objects.objects_vector[0].roi.width < 475 &&
      srv->objects.objects_vector[0].roi.height > 345 &&
      srv->objects.objects_vector[0].roi.height < 360);
  } else {
    auto node = rclcpp::Node::make_shared("movidius_ncs_image_tests");

    auto client = node->create_client<object_msgs::srv::ClassifyObject>("movidius_ncs_image/"
        "classify_object");
    auto request = std::make_shared<object_msgs::srv::ClassifyObject::Request>();
    request->image_path = "/opt/movidius/ncappzoo/data/images/cat.jpg";

    if (!client->wait_for_service(std::chrono::seconds(20))) {
      ASSERT_TRUE(false) << "service not available after waiting";
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    auto ret = rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(5));
    EXPECT_EQ(ret, rclcpp::executor::FutureReturnCode::SUCCESS);

    auto srv = result.get();

    EXPECT_TRUE(srv->objects.objects_vector.size());
    EXPECT_TRUE(srv->objects.objects_vector[0].object_name.find("cat") != std::string::npos);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  system("movidius_ncs_image &");
  getCnnType();
  int ret = RUN_ALL_TESTS();
  system("killall movidius_ncs_image &");
  rclcpp::shutdown();
  return ret;
}
