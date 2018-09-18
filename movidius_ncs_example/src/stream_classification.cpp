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
#include <object_msgs/msg/objects.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#define LINESPACING 20

class ClassificationShow : public rclcpp::Node
{
public:
  ClassificationShow()
  : Node("classification_show")
  {
    rclcpp::Node::SharedPtr node = std::shared_ptr<rclcpp::Node>(this); 
    cam_sub_ = std::make_unique<camSub>(node, "/camera/color/image_raw");
    obj_sub_ = std::make_unique<objSub>(node, "/movidius_ncs_stream/classified_objects");
    sync_sub_ = std::make_unique<sync>(*cam_sub_, *obj_sub_, 10);
    sync_sub_->registerCallback(&ClassificationShow::showImage, this);
  }

private:
  using camSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using objSub = message_filters::Subscriber<object_msgs::msg::Objects>;
  using sync =
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, object_msgs::msg::Objects>;
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
    const object_msgs::msg::Objects::SharedPtr & objs)
  {
    cv::Mat cvImage = cv_bridge::toCvShare(img, "bgr8")->image;
    int cnt = 0;

    for (auto obj : objs->objects_vector) {
      std::stringstream ss;
      ss << obj.object_name << ": " << obj.probability * 100 << '%';
      cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)),
        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
    }

    std::stringstream ss;
    int fps = getFPS();
    ss << "FPS: " << fps;
    cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)),
      cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));

    cv::imshow("image_viewer", cvImage);
    cv::waitKey(5);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClassificationShow>());
  rclcpp::shutdown();
  return 0;
}
