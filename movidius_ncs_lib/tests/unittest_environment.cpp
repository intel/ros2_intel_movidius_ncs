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

TEST(UnitTestEnvironment, testSDK)
{
  std::system("python2 /opt/movidius/NCSDK/tests/api-check/ncs-python2-check.py > /tmp/NCS_unittest.txt");
  EXPECT_TRUE(std::ifstream("/tmp/NCS_unittest.txt").is_open());
  std::ifstream fin("/tmp/NCS_unittest.txt");
  std::string ncs_check, tmp;
  while (std::getline(fin, tmp))
  {
    ncs_check = tmp;
  }
  EXPECT_EQ(ncs_check, "NCS device working.");
  std::system("rm -rf /tmp/NCS_unittest.txt > /dev/null 2>&1");
}

TEST(UnitTestEnvironment, testAppZoo)
{
  EXPECT_TRUE(std::ifstream("/opt/movidius/ncappzoo").is_open());
  std::system("cd /opt/movidius/ncappzoo && make > /dev/null 2>&1");
  std::vector<std::string> caffe_dirs = { "AlexNet", "GoogLeNet", "SqueezeNet", "SSD_MobileNet", "TinyYolo" };
  std::vector<std::string> tf_dirs = { "inception_v1", "inception_v2", "inception_v3", "inception_v4", "mobilenets/model" };
  for (std::string caffe : caffe_dirs)
  {
    EXPECT_TRUE(std::ifstream("/opt/movidius/ncappzoo/caffe/" + caffe + "/graph").is_open());
  }

  for (std::string tf : tf_dirs)
  {
    EXPECT_TRUE(std::ifstream("/opt/movidius/ncappzoo/tensorflow/" + tf + "/graph").is_open());
  }
}

TEST(UnitTestEnvironment, testCategories)
{

  EXPECT_TRUE(std::ifstream("/opt/movidius/ncappzoo/data/ilsvrc12/imagenet1000.txt").is_open());
  EXPECT_TRUE(std::ifstream("/opt/movidius/ncappzoo/data/ilsvrc12/imagenet1001.txt").is_open());
  EXPECT_TRUE(std::ifstream("/opt/movidius/ncappzoo/data/ilsvrc12/voc20.txt").is_open());
  EXPECT_TRUE(std::ifstream("/opt/movidius/ncappzoo/data/ilsvrc12/voc21.txt").is_open());

}	


int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

