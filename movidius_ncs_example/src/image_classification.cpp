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

#include <object_msgs/srv/classify_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("movidius_ncs_example");

  if (argc != 2) {
    RCLCPP_INFO(node->get_logger(),
      "Usage: ros2 run movidius_ncs_example movidius_ncs_example_image_classification "
      "<image_path>");
    return -1;
  }

  auto client = node->create_client<object_msgs::srv::ClassifyObject>("movidius_ncs_image/"
      "classify_object");
  auto request = std::make_shared<object_msgs::srv::ClassifyObject::Request>();
  request->image_path = argv[1];

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    auto srv = result.get();
    for (unsigned int i = 0; i < srv->objects.objects_vector.size(); i++) {
      RCLCPP_INFO(node->get_logger(), "%d: object: %s\nprobability: %lf%%", i,
        srv->objects.objects_vector[i].object_name.c_str(),
        srv->objects.objects_vector[i].probability * 100);
    }
    RCLCPP_INFO(node->get_logger(), "inference time: %fms", srv->objects.inference_time_ms);
  }

  return 0;
}
