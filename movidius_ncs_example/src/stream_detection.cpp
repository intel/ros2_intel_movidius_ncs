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

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#define LINESPACING 20

class DetectionShow : public rclcpp::Node
{
public:
  DetectionShow()
  : Node("detection_show")
  {
    cam_sub_ = std::make_unique<camSub>(this, "/camera/color/image_raw");
    obj_sub_ = std::make_unique<objSub>(this, "/movidius_ncs_stream/detected_objects");
    sync_sub_ = std::make_unique<sync>(*cam_sub_, *obj_sub_, 10);
    sync_sub_->registerCallback(&DetectionShow::showImage, this);
  }

private:
  using camSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using objSub = message_filters::Subscriber<object_msgs::msg::ObjectsInBoxes>;
  using sync =
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, object_msgs::msg::ObjectsInBoxes>;
  std::unique_ptr<camSub> cam_sub_;
  std::unique_ptr<objSub> obj_sub_;
  std::unique_ptr<sync> sync_sub_;

  int getFPS()
  {
    static int fps = 0;
    static boost::posix_time::ptime duration_start =
      boost::posix_time::microsec_clock::local_time();
    static int frame_cnt = 0;

    frame_cnt++;

    boost::posix_time::ptime current = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration msdiff = current - duration_start;

    if (msdiff.total_milliseconds() > 1000) {
      fps = frame_cnt;
      frame_cnt = 0;
      duration_start = current;
    }

    return fps;
  }

  void showImage(
    const sensor_msgs::msg::Image::SharedPtr & img,
    const object_msgs::msg::ObjectsInBoxes::SharedPtr & objs)
  {
    cv::Mat cvImage = cv_bridge::toCvShare(img, "bgr8")->image;
    int width = img->width;
    int height = img->height;

    for (auto obj : objs->objects_vector) {
      std::stringstream ss;
      ss << obj.object.object_name << ": " << obj.object.probability * 100 << '%';

      int x = obj.roi.x_offset;
      int y = obj.roi.y_offset;
      int w = obj.roi.width;
      int h = obj.roi.height;

      int xmin = ((x - w / 2) > 0) ? (x - w / 2) : 0;
      int xmax = ((x + w / 2) < width) ? (x + w / 2) : width;
      int ymin = ((y - h / 2) > 0) ? (y - h / 2) : 0;
      int ymax = ((y + h / 2) < height) ? (y + h / 2) : height;

      cv::Point left_top = cv::Point(xmin, ymin);
      cv::Point right_bottom = cv::Point(xmax, ymax);
      cv::rectangle(cvImage, left_top, right_bottom, cv::Scalar(0, 255, 0), 1, 8, 0);
      cv::rectangle(cvImage, cvPoint(xmin, ymin), cvPoint(xmax, ymin + 20), cv::Scalar(0, 255, 0),
        -1);
      cv::putText(cvImage, ss.str(), cvPoint(xmin + 5, ymin + 20), cv::FONT_HERSHEY_PLAIN, 1,
        cv::Scalar(0, 0, 255), 1);
    }
    std::stringstream ss;
    int fps = getFPS();
    ss << "FPS: " << fps;
    cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING), cv::FONT_HERSHEY_PLAIN, 1,
      cv::Scalar(0, 255, 0));

    cv::imshow("image_viewer", cvImage);
    cv::waitKey(5);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionShow>());
  rclcpp::shutdown();
  return 0;
}
