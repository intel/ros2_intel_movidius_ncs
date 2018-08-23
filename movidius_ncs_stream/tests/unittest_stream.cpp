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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <functional>
#include <memory>
#include <chrono>
#include <stdexcept>
#include <exception>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rate.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "movidius_ncs_lib/param.hpp"

bool test_pass = false;
bool cnn_type_flag = false;
#define MAXSIZE 100
const char * buffer;
std::promise<bool> sub_called;

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

void callback_classify(
  const sensor_msgs::msg::Image::SharedPtr & img,
  const object_msgs::msg::Objects::SharedPtr & msg)
{
  test_pass = true;
  sub_called = std::promise<bool>(std::allocator_arg, std::allocator<bool>());
  sub_called.set_value(true);
}

void callback_detect(
  const sensor_msgs::msg::Image::SharedPtr & img,
  const object_msgs::msg::ObjectsInBoxes::SharedPtr & msg)
{
  test_pass = true;
  sub_called = std::promise<bool>(std::allocator_arg, std::allocator<bool>());
  sub_called.set_value(true);
}

TEST(UnitTestStream, testStream) {
  auto node = rclcpp::Node::make_shared("movidius_ncs_stream_tests");
  std::shared_future<bool> sub_called_future(sub_called.get_future());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  {
    typedef message_filters::sync_policies::ApproximateTime
      <sensor_msgs::msg::Image, object_msgs::msg::Objects> approximatePolicy_cla;
    typedef message_filters::sync_policies::ApproximateTime
      <sensor_msgs::msg::Image, object_msgs::msg::ObjectsInBoxes> approximatePolicy_dec;

    message_filters::Subscriber<sensor_msgs::msg::Image> camSub(node.get(),
      "/camera/color/image_raw");
    message_filters::Subscriber<object_msgs::msg::Objects> objSub_cla(node.get(),
      "/movidius_ncs_stream/classified_objects");
    message_filters::Synchronizer<approximatePolicy_cla> sync_cla(
      approximatePolicy_cla(100), camSub, objSub_cla);
    sync_cla.registerCallback(callback_classify);

    message_filters::Subscriber<object_msgs::msg::ObjectsInBoxes> objSub_dec(node.get(),
      "/movidius_ncs_stream/detected_objects");
    message_filters::Synchronizer<approximatePolicy_dec> sync_dec(
      approximatePolicy_dec(100), camSub, objSub_dec);
    sync_dec.registerCallback(callback_detect);

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
