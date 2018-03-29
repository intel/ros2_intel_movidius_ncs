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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"

#include <object_msgs/msg/objects.hpp>

#define LINESPACING 20

int encoding2mat_type(const std::string& encoding)
{
  if (encoding == "mono8")
  {
    return CV_8UC1;
  }
  else if (encoding == "bgr8")
  {
    return CV_8UC3;
  }
  else if (encoding == "mono16")
  {
    return CV_16SC1;
  }
  else if (encoding == "rgba8")
  {
    return CV_8UC4;
  }
  else if (encoding == "bgra8")
  {
    return CV_8UC4;
  }
  else if (encoding == "32FC1")
  {
    return CV_32FC1;
  }
  else if (encoding == "rgb8")
  {
    return CV_8UC3;
  }
  else
  {
    throw std::runtime_error("Unsupported encoding type");
  }
}

void show_image(const object_msgs::msg::Objects::SharedPtr& msg)
{
  cv::Mat frame(msg->image.height, msg->image.width, encoding2mat_type(msg->image.encoding),
                const_cast<unsigned char*>(msg->image.data.data()), msg->image.step);

  cv::Mat cvframe;
  if (msg->image.encoding == "rgb8")
  {
    cv::Mat frame2;
    cv::cvtColor(frame, frame2, cv::COLOR_RGB2BGR);
    cvframe = frame2;
  }
  else
  {
    cvframe = frame;
  }
  int cnt = 0;

  for (auto obj : msg->objects_vector)
  {
    std::stringstream ss;
    ss << obj.object_name << ": " << obj.probability * 100 << '%';
    cv::putText(cvframe, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
  }

  // std::stringstream ss;
  // ss << "FPS: " << msg->fps;
  // cv::putText(cvframe, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)),
  //             cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
  cv::imshow("image_viewer", cvframe);
  cv::waitKey(5);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("movidius_ncs_example");

  // Initialize a subscriber that will receive the ROS Image message to be displayed.
  auto sub = node->create_subscription<object_msgs::msg::Objects>(
      "/movidius_ncs_stream/classified_objects",
      [](const object_msgs::msg::Objects::SharedPtr msg) -> void { show_image(msg); });

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
