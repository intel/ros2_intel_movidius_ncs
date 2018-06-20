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

#include <string>
#include <fstream>
#include <iostream>
#include <cstdio>

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <object_msgs/msg/objects.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include "ament_index_cpp/get_resource.hpp"
#include "movidius_ncs_lib/param.hpp"

static bool test_pass = false;
bool cnn_type_flag = false;
void getCnnType()
{
    std::string content;
    std::string prefix_path;
    std::string line;
    std::string param_file;
    char composition_path[100];
    ament_index_cpp::get_resource("packages", "movidius_ncs_launch", content, &prefix_path);
    std::ifstream fin(prefix_path + "/share/movidius_ncs_launch/config/default.yaml");
    if(!std::ifstream("./../../../install/bin/api_composition").is_open())
    {
       snprintf(composition_path,100,"cp %s/lib/composition/api_composition %s/bin","./../../../install","./../../../install");
       system(composition_path);
    }
    if (std::getline(fin, line))
    {
        param_file = line.substr(line.find(":") + 1);
    }
    auto param_ = std::make_shared<movidius_ncs_lib::Param>();
    param_file.erase(param_file.begin(),
                       std::find_if(param_file.begin(), param_file.end(),
                                    std::not1(std::ptr_fun<int, int>(std::isspace))));
    if (param_->loadParamFromYAML(prefix_path + "/share/movidius_ncs_launch/config/" + param_file))
    {
        if (!param_->cnn_type_.compare("alexnet") || !param_->cnn_type_.compare("googlenet") ||
            !param_->cnn_type_.compare("inception_v1") || !param_->cnn_type_.compare("inception_v2") ||
            !param_->cnn_type_.compare("inception_v3") || !param_->cnn_type_.compare("inception_v4") ||
            !param_->cnn_type_.compare("mobilenet") || !param_->cnn_type_.compare("squeezenet"))
        {
              cnn_type_flag=false;
        }
        else
        {
              cnn_type_flag=true;
        }
    } 
}

void show_image_classcification(const object_msgs::msg::Objects::SharedPtr msg)
{

    test_pass=true;
    EXPECT_TRUE(true);
    rclcpp::shutdown();
}

void show_image_detection(const object_msgs::msg::ObjectsInBoxes::SharedPtr msg)
{

    test_pass=true;
    EXPECT_TRUE(true);
    rclcpp::shutdown();
}

TEST(UnitTestStream, testStream)
{
  getCnnType();
  if(cnn_type_flag)
  {
      auto node = rclcpp::Node::make_shared("movidius_ncs_stream_tests");

      auto sub = node->create_subscription<object_msgs::msg::ObjectsInBoxes>(
         "/movidius_ncs_stream/detected_objects",
[](const object_msgs::msg::ObjectsInBoxes::SharedPtr msg) -> void { show_image_detection(msg); });
      rclcpp::spin(node);
      rclcpp::shutdown();
      EXPECT_TRUE(test_pass);
  }
  else
  {
      auto node = rclcpp::Node::make_shared("movidius_ncs_stream_tests");

      auto sub = node->create_subscription<object_msgs::msg::Objects>(
           "/movidius_ncs_stream/classified_objects",
[](const object_msgs::msg::Objects::SharedPtr msg) -> void { show_image_classcification(msg); });
      rclcpp::spin(node);
      rclcpp::shutdown();
      EXPECT_TRUE(test_pass);

  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  system("realsense_ros2_camera &");
  system("api_composition &");
  system("launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_stream_launch.py &");
  int ret=RUN_ALL_TESTS();
  system("killall api_composition &");
  system("killall realsense_ros2_camera &");
  return ret;
}
