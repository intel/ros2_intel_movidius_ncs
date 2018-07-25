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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <object_msgs/msg/objects.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>

#include <string>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <functional>
#include <memory>
#include <chrono>
#include <stdexcept>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rate.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "movidius_ncs_lib/param.hpp"

bool test_pass = false;
bool cnn_type_flag = false;
#define MAXSIZE 100
const char * buffer;

template<typename DurationT>
void wait_for_future(
  rclcpp::executor::Executor & executor,
  std::shared_future<bool> & future,
  const DurationT & timeout)
{
  using rclcpp::executor::FutureReturnCode;
  rclcpp::executor::FutureReturnCode future_ret;
  auto start_time = std::chrono::steady_clock::now();
  future_ret = executor.spin_until_future_complete(future, timeout);
  auto elapsed_time = std::chrono::steady_clock::now() - start_time;
  EXPECT_EQ(FutureReturnCode::SUCCESS, future_ret) <<
    "the usb camera don't publish data to topic\n" <<
    "future failed to be set after: " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
    " milliseconds\n";
}

TEST(UnitTestStream, testStream) {
  auto node = rclcpp::Node::make_shared("movidius_ncs_stream_tests");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  std::promise<bool> sub_called;
  std::shared_future<bool> sub_called_future(sub_called.get_future());

  auto callback_classify =
    [&sub_called](const object_msgs::msg::Objects::SharedPtr msg) -> void
    {
      test_pass = true;
      sub_called.set_value(true);
    };

  auto callback_detect =
    [&sub_called](const object_msgs::msg::ObjectsInBoxes::SharedPtr msg) -> void
    {
      test_pass = true;
      sub_called.set_value(true);
    };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  {
    auto sub1 = node->create_subscription<object_msgs::msg::ObjectsInBoxes>(
      "/movidius_ncs_stream/detected_objects",
      callback_detect, custom_qos_profile);

    auto sub2 = node->create_subscription<object_msgs::msg::Objects>(
      "/movidius_ncs_stream/classified_objects",
      callback_classify, custom_qos_profile);

    executor.spin_once(std::chrono::seconds(0));

    wait_for_future(executor, sub_called_future, std::chrono::seconds(10));

    EXPECT_TRUE(test_pass);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto offset = std::chrono::seconds(10);
  system("ros2 run composition api_composition &");
  system("launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/"
    "launch/ncs_stream_launch.py &");
  rclcpp::sleep_for(offset);
  int ret = RUN_ALL_TESTS();
  system("killall api_composition &");
  system("killall realsense_ros2_camera &");
  rclcpp::shutdown();
  return ret;
}
