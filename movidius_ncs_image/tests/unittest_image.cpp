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
#include <gtest/gtest.h>
#include <fstream>


#include "rclcpp/rclcpp.hpp"

#include <object_msgs/srv/detect_object.hpp>
#include <object_msgs/srv/classify_object.hpp>
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
TEST(UnitTestImage, testImage)
{
  getCnnType();
  if(cnn_type_flag)
  {
    auto node = rclcpp::Node::make_shared("movidius_ncs_image_tests");
    auto client = node->create_client<object_msgs::srv::DetectObject>("/movidius_ncs_image/detect_object");
  
    auto request = std::make_shared<object_msgs::srv::DetectObject::Request>();
    std::vector<std::string> image_format = {".jpg", ".jpeg", ".png", ".bmp"};
    std::string content;
    std::string prefix_path;
    ament_index_cpp::get_resource("packages", "movidius_ncs_lib", content, &prefix_path);
    for (std::string suffix : image_format)
    {
       request->image_path = prefix_path+"/../src/ros2_intel_movidius_ncs/data/images/bicycle"+suffix;
       auto result = client->async_send_request(request);

       EXPECT_EQ(rclcpp::spin_until_future_complete(node, result),rclcpp::executor::FutureReturnCode::SUCCESS);
       auto srv = result.get();

       EXPECT_TRUE(srv->objects.objects_vector.size());
       EXPECT_EQ(srv->objects.objects_vector[0].object.object_name, "bicycle");
       RCLCPP_INFO(
          node->get_logger(), "location: (%d, %d, %d, %d)",
          srv->objects.objects_vector[0].roi.x_offset, srv->objects.objects_vector[0].roi.y_offset,
          srv->objects.objects_vector[0].roi.width, srv->objects.objects_vector[0].roi.height);
        EXPECT_TRUE(srv->objects.objects_vector[0].roi.x_offset > 360 &&
           srv->objects.objects_vector[0].roi.x_offset < 375 &&
           srv->objects.objects_vector[0].roi.y_offset > 265 &&
           srv->objects.objects_vector[0].roi.y_offset < 280 &&
           srv->objects.objects_vector[0].roi.width > 460 &&
           srv->objects.objects_vector[0].roi.width < 475 &&
           srv->objects.objects_vector[0].roi.height > 345 &&
           srv->objects.objects_vector[0].roi.height < 360);
     }
  }
  else
  {
     auto node = rclcpp::Node::make_shared("movidius_ncs_image_tests");
     auto client = node->create_client<object_msgs::srv::ClassifyObject>("/movidius_ncs_image/classify_object");
     auto request = std::make_shared<object_msgs::srv::ClassifyObject::Request>();
     request->image_path = "/opt/movidius/ncappzoo/data/images/cat.jpg";
     auto result = client->async_send_request(request);
     if(rclcpp::spin_until_future_complete(node, result) != rclcpp::executor::FutureReturnCode::SUCCESS)
     {
          RCLCPP_ERROR(node->get_logger(), "service call failed :(")
     }
     auto srv = result.get();
     EXPECT_EQ(rclcpp::spin_until_future_complete(node, result),rclcpp::executor::FutureReturnCode::SUCCESS);
     EXPECT_TRUE(srv->objects.objects_vector.size());
    EXPECT_TRUE(srv->objects.objects_vector[0].object_name.find("cat") != std::string::npos);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  system("movidius_ncs_image &");
  int ret=RUN_ALL_TESTS();
  system("killall movidius_ncs_image &");
  return ret;
}
