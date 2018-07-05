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

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <object_msgs/srv/detect_object.hpp>

#include <chrono>
#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("movidius_ncs_example");
  if (argc != 2) {
    RCLCPP_INFO(node->get_logger(),
      "Usage: ros2 run movidius_ncs_example movidius_ncs_example_image_detection "
      "<image_path>");
    return -1;
  }

  auto client =
    node->create_client<object_msgs::srv::DetectObject>("movidius_ncs_image/detect_object");
  auto request = std::make_shared<object_msgs::srv::DetectObject::Request>();
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
    cv::Mat image = cv::imread(argv[1]);
    int width = image.cols;
    int height = image.rows;
    for (unsigned int i = 0; i < srv->objects.objects_vector.size(); i++) {
      std::stringstream ss;
      ss << srv->objects.objects_vector[i].object.object_name << ": " <<
        srv->objects.objects_vector[i].object.probability * 100 << "%";
      RCLCPP_INFO(node->get_logger(), "%d: object: %s", i,
        srv->objects.objects_vector[i].object.object_name.c_str());
      RCLCPP_INFO(node->get_logger(), "prob: %f",
        srv->objects.objects_vector[i].object.probability);
      RCLCPP_INFO(
        node->get_logger(), "location: (%d, %d, %d, %d)",
        srv->objects.objects_vector[i].roi.x_offset, srv->objects.objects_vector[i].roi.y_offset,
        srv->objects.objects_vector[i].roi.width, srv->objects.objects_vector[i].roi.height);

      int x = srv->objects.objects_vector[i].roi.x_offset;
      int y = srv->objects.objects_vector[i].roi.y_offset;
      int w = srv->objects.objects_vector[i].roi.width;
      int h = srv->objects.objects_vector[i].roi.height;

      int xmin = ((x - w / 2) > 0) ? (x - w / 2) : 0;
      int xmax = ((x + w / 2) < width) ? (x + w / 2) : width;
      int ymin = ((y - h / 2) > 0) ? (y - h / 2) : 0;
      int ymax = ((y + h / 2) < height) ? (y + h / 2) : height;

      cv::Point left_top = cv::Point(xmin, ymin);
      cv::Point right_bottom = cv::Point(xmax, ymax);
      cv::rectangle(image, left_top, right_bottom, cv::Scalar(0, 255, 0), 1, 8, 0);
      cv::rectangle(image, cvPoint(xmin, ymin), cvPoint(xmax, ymin + 20), cv::Scalar(0, 255, 0),
        -1);
      cv::putText(image, ss.str(), cvPoint(xmin + 5, ymin + 20), cv::FONT_HERSHEY_PLAIN, 1,
        cv::Scalar(0, 0, 255), 1);
    }
    RCLCPP_INFO(node->get_logger(), "inference time: %fms", srv->objects.inference_time_ms);
    cv::imshow("image_detection", image);
    cv::waitKey(0);
  }
  return 0;
}
