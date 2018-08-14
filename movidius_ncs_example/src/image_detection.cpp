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

#include <dirent.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <object_msgs/srv/detect_object.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <vector>

#define MOVEWINDOW 1000
#define DEFAULT_PARALLEL_SIZE 1
#define DEFAULT_IMAGE_BASE_PATH "/opt/movidius/ncappzoo/data/images/"
#define DEFAULT_DEMO_MODE 0
#define DEFAULT_PARALLEL_FLAG 1

std::vector<std::string> getImagePath(std::string image_dir)
{
  if (image_dir.back() != '/') {
    image_dir += "/";
  }

  std::vector<std::string> files;

  DIR * dir;
  struct dirent * ptr;

  if ((dir = opendir(image_dir.c_str())) == NULL) {
    std::cerr << "Open Dir error..." << std::endl;
    exit(1);
  }

  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) {
      continue;
    } else if (ptr->d_type == DT_REG) {
      files.push_back(image_dir + ptr->d_name);
    }
  }
  closedir(dir);

  return files;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("movidius_ncs_example");
  if (argc != 2) {
    RCLCPP_INFO(node->get_logger(),
      "Usage: ros2 run movidius_ncs_example "
      "movidius_ncs_example_image_detection "
      "<image_path>");
    return -1;
  }

  int parallel_flag = DEFAULT_PARALLEL_FLAG;
  int parallel_size = DEFAULT_PARALLEL_SIZE;
  int demo_mode = DEFAULT_DEMO_MODE;
  std::string image_base_path = DEFAULT_IMAGE_BASE_PATH;
  std::vector<std::string> image_paths = getImagePath(image_base_path);

  rclcpp::Client<object_msgs::srv::DetectObject>::SharedPtr client;
  if (parallel_flag == 0) {
    client = node->create_client<object_msgs::srv::DetectObject>(
      "movidius_ncs_image/"
      "detect_object_single");
  } else {
    client = node->create_client<object_msgs::srv::DetectObject>(
      "movidius_ncs_image/"
      "detect_object");
  }

  auto request = std::make_shared<object_msgs::srv::DetectObject::Request>();

  if (demo_mode == 0) {
    request->image_paths = image_paths;
    boost::posix_time::ptime start =
      boost::posix_time::microsec_clock::local_time();
    std::cout << "start = " << start << std::endl;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(),
          "Interrupted while waiting for the service. Exiting.");
        return 1;
      }
      RCLCPP_INFO(node->get_logger(),
        "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      boost::posix_time::ptime end =
        boost::posix_time::microsec_clock::local_time();
      std::cout << "end = " << end << std::endl;
      boost::posix_time::time_duration msdiff = end - start;

      auto srv = result.get();

      for (unsigned int i = 0; i < srv->objects.size(); i++) {
        cv_bridge::CvImage cv_image;
        cv_image.image = cv::imread(image_paths[i]);
        cv_image.encoding = "bgr8";
        int width = cv_image.image.cols;
        int height = cv_image.image.rows;

        RCLCPP_INFO(node->get_logger(), "Detection result for image No.%u:",
          i + 1);
        for (unsigned int j = 0; j < srv->objects[i].objects_vector.size();
          j++)
        {
          std::stringstream ss;
          ss << srv->objects[i].objects_vector[j].object.object_name << ": " <<
            srv->objects[i].objects_vector[j].object.probability * 100 << "%";

          RCLCPP_INFO(
            node->get_logger(), "%d: object: %s", j,
            srv->objects[i].objects_vector[j].object.object_name.c_str());
          RCLCPP_INFO(node->get_logger(), "prob: %f",
            srv->objects[i].objects_vector[j].object.probability);
          RCLCPP_INFO(node->get_logger(), "location: (%d, %d, %d, %d)",
            srv->objects[i].objects_vector[j].roi.x_offset,
            srv->objects[i].objects_vector[j].roi.y_offset,
            srv->objects[i].objects_vector[j].roi.width,
            srv->objects[i].objects_vector[j].roi.height);

          int xmin = srv->objects[i].objects_vector[j].roi.x_offset;
          int ymin = srv->objects[i].objects_vector[j].roi.y_offset;
          int w = srv->objects[i].objects_vector[j].roi.width;
          int h = srv->objects[i].objects_vector[j].roi.height;

          int xmax = ((xmin + w) < width) ? (xmin + w) : width;
          int ymax = ((ymin + h) < height) ? (ymin + h) : height;

          cv::Point left_top = cv::Point(xmin, ymin);
          cv::Point right_bottom = cv::Point(xmax, ymax);
          cv::rectangle(cv_image.image, left_top, right_bottom,
            cv::Scalar(0, 255, 0), 1, cv::LINE_8, 0);
          cv::rectangle(cv_image.image, cvPoint(xmin, ymin),
            cvPoint(xmax, ymin + 20), cv::Scalar(0, 255, 0), -1);
          cv::putText(cv_image.image, ss.str(), cvPoint(xmin + 5, ymin + 20),
            cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 1);
        }
        cv::imshow("image_detection", cv_image.image);
        cv::waitKey(0);
      }
      RCLCPP_INFO(node->get_logger(), "inference %lu images during %ld ms",
        srv->objects.size(), msdiff.total_milliseconds());
    }
  } else {
    while (1) {
      std::vector<int> random_index_list;
      for (int i = 0; i < parallel_size; i++) {
        std::random_device rd;
        std::default_random_engine engine(rd());
        std::uniform_int_distribution<> dis(0, image_paths.size() - 1);
        auto dice = std::bind(dis, engine);
        int random_index = dice();
        random_index_list.push_back(random_index);
        request->image_paths.push_back(image_paths[random_index]);
      }

      while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(),
            "Interrupted while waiting for the service. Exiting.");
          return 1;
        }
        RCLCPP_INFO(node->get_logger(),
          "service not available, waiting again...");
      }

      boost::posix_time::ptime start =
        boost::posix_time::microsec_clock::local_time();
      auto result = client->async_send_request(request);

      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        boost::posix_time::ptime end =
          boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration msdiff = end - start;

        auto srv = result.get();
        for (unsigned int i = 0; i < srv->objects.size(); i++) {
          cv_bridge::CvImage cv_image;
          cv_image.image = cv::imread(image_paths[random_index_list[i]]);
          cv_image.encoding = "bgr8";
          int width = cv_image.image.cols;
          int height = cv_image.image.rows;

          for (unsigned int j = 0; j < srv->objects[i].objects_vector.size();
            j++)
          {
            std::stringstream ss;
            ss << srv->objects[i].objects_vector[j].object.object_name <<
              ": " << srv->objects[i].objects_vector[j].object.probability * 100 <<
              "%";

            int xmin = srv->objects[i].objects_vector[j].roi.x_offset;
            int ymin = srv->objects[i].objects_vector[j].roi.y_offset;
            int w = srv->objects[i].objects_vector[j].roi.width;
            int h = srv->objects[i].objects_vector[j].roi.height;

            int xmax = ((xmin + w) < width) ? (xmin + w) : width;
            int ymax = ((ymin + h) < height) ? (ymin + h) : height;

            cv::Point left_top = cv::Point(xmin, ymin);
            cv::Point right_bottom = cv::Point(xmax, ymax);
            cv::rectangle(cv_image.image, left_top, right_bottom,
              cv::Scalar(0, 255, 0), 1, cv::LINE_8, 0);
            cv::rectangle(cv_image.image, cvPoint(xmin, ymin),
              cvPoint(xmax, ymin + 20), cv::Scalar(0, 255, 0), -1);

            cv::putText(cv_image.image, ss.str(), cvPoint(xmin + 5, ymin + 20),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 1);
          }

          if (parallel_flag == 0) {
            cv::imshow("image detection with single device", cv_image.image);
            cv::waitKey(1);
          } else {
            cv::namedWindow("image detection with multiple devices");
            cv::moveWindow("image detection with multiple devices", MOVEWINDOW,
              0);
            cv::imshow("image detection with multiple devices", cv_image.image);
            cv::waitKey(1);
          }
        }
        RCLCPP_INFO(node->get_logger(), "inference %lu images during %ld ms",
          srv->objects.size(), msdiff.total_milliseconds());
      }
    }
  }
  return 0;
}
