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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include <opencv2/highgui.hpp>


#include "movidius_ncs_lib/ncs.hpp"
#include "movidius_ncs_lib/param.hpp"
#include "ament_index_cpp/get_resource.hpp"


TEST(UnitTestFunction, testLibraryFunctions)
{
  std::vector<std::string> caffe_dirs = { "AlexNet", "GoogLeNet", "SqueezeNet", "SSD_MobileNet", "TinyYolo" };
  std::vector<std::string> tf_dirs = { "inception_v1", "inception_v2", "inception_v3", "inception_v4", "mobilenets/model" };
  std::vector<std::string> caffe_nets = { "alexnet", "googlenet", "squeezenet", "mobilenetssd", "tinyyolo_v1" };
  std::vector<std::string> tf_nets = { "inception_v1", "inception_v2", "inception_v3", "inception_v4", "mobilenet" };
  std::vector<std::string> caffe_categories = { "imagenet1000.txt", "imagenet1000.txt", "imagenet1001.txt", "voc21.txt", "voc20.txt" };
  std::vector<std::vector<float>> caffe_means = { { 104.0069879317889, 116.66876761696767, 122.6789143406786 },
                                                  { 104.0069879317889, 116.66876761696767, 122.6789143406786 },
                                                  { 104.0069879317889, 116.66876761696767, 122.6789143406786 },
                                                  { 127.5, 127.5, 127.5 },
                                                  { 0, 0, 0 } };

  std::vector<std::vector<float>> tf_means = { { 128.0, 128.0, 128.0 },
                                               { 128.0, 128.0, 128.0 },
                                               { 128.0, 128.0, 128.0 },
                                               { 128.0, 128.0, 128.0 },
                                               { 127.5, 127.5, 127.5 } };
  std::vector<float> caffe_scales = { 1.0, 1.0, 1.0, 0.007843, 0.00392156 };
  std::vector<float> tf_scales = { 0.0078125, 0.0078125, 0.0078125, 0.0078125, 0.007843 };
  std::vector<int> caffe_dimensions = { 227, 224, 227, 300, 448 };
  std::vector<int> tf_dimensions = { 224, 224, 299, 299, 224 };

  for (unsigned int i = 0; i < caffe_nets.size(); i++)
  {
    ASSERT_NO_THROW({
      std::shared_ptr<movidius_ncs_lib::NCS> handle = std::make_shared<movidius_ncs_lib::NCS>(
          0, static_cast<movidius_ncs_lib::Device::LogLevel>(0), caffe_nets[i],
          "/opt/movidius/ncappzoo/caffe/" + caffe_dirs[i] + "/graph",
          "/opt/movidius/ncappzoo/data/ilsvrc12/" + caffe_categories[i], caffe_dimensions[i], caffe_means[i],
          caffe_scales[i], 3);
      EXPECT_TRUE(handle != nullptr);
      std::string file_path;
      std::string content;
      std::string prefix_path;
      ament_index_cpp::get_resource("packages", "movidius_ncs_launch", content, &prefix_path);
      file_path=prefix_path+"/share/movidius_ncs_launch/config/"+ caffe_nets[i] + ".yaml";
      std::shared_ptr<movidius_ncs_lib::Param> Paramhandle = std::make_shared<movidius_ncs_lib::Param>();
      EXPECT_TRUE(Paramhandle->loadParamFromYAML(file_path));
      EXPECT_TRUE(Paramhandle->validateParam());
    });
  }

  for (unsigned int i = 0; i < tf_nets.size(); i++)
  {
    ASSERT_NO_THROW({
      std::shared_ptr<movidius_ncs_lib::NCS> handle = std::make_shared<movidius_ncs_lib::NCS>(
          0, static_cast<movidius_ncs_lib::Device::LogLevel>(0), tf_nets[i],
          "/opt/movidius/ncappzoo/tensorflow/" + tf_dirs[i] + "/graph",
          "/opt/movidius/ncappzoo/data/ilsvrc12/imagenet1001.txt", tf_dimensions[i], tf_means[i], tf_scales[i], 3);
      EXPECT_TRUE(handle != nullptr);
      std::string file_path;
      std::string content;
      std::string prefix_path;
      ament_index_cpp::get_resource("packages", "movidius_ncs_launch", content, &prefix_path);
      file_path=prefix_path+"/share/movidius_ncs_launch/config/"+ tf_nets[i] + ".yaml";
      std::shared_ptr<movidius_ncs_lib::Param> Paramhandle = std::make_shared<movidius_ncs_lib::Param>();
      EXPECT_TRUE(Paramhandle->loadParamFromYAML(file_path));
      EXPECT_TRUE(Paramhandle->validateParam());
      sleep(10);
    });
  }
}

TEST(UnitTestFunction, testLibraryIncorrectInputs)
{
  try
  {
    std::vector<float> incorrect_mean = { 0, 0, 0 };
    std::shared_ptr<movidius_ncs_lib::NCS> handle = std::make_shared<movidius_ncs_lib::NCS>(
        0, static_cast<movidius_ncs_lib::Device::LogLevel>(0), "Incorrect_type", "graph_not_exist",
        "categories_not_exist", 0, incorrect_mean, 0, 3);
  }
  catch (...)
  {
    SUCCEED();
  }
}


int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

