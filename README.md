# ros2_intel_movidius_ncs

## 1 Introduction
The Movidius™ Neural Compute Stick ([NCS](https://developer.movidius.com/)) is a tiny fanless deep learning device that you can use to learn AI programming at the edge. NCS is powered by the same low power high performance Movidius™ Vision Processing Unit ([VPU](https://www.movidius.com/solutions/vision-processing-unit)) that can be found in millions of smart security cameras, gesture controlled drones, industrial machine vision equipment, and more.  

This project is a ROS2 wrapper for NC API of [NCSDK](https://movidius.github.io/ncsdk/), providing the following features:
* A ROS2 service for object classification and detection of a static image file
* A ROS2 publisher for object classification and detection of a video stream from a RGB camera
* Demo applications to show the capabilities of ROS2 service and publisher
* Support multiple CNN models of Caffe and Tensorflow
  
## 2 Prerequisite
* An x86_64 computer running Ubuntu 16.04. OS X and Windows are not supported yet
* ROS2 Bouncy
* Movidius Neural Compute Stick (NCS)
* Movidius Neural Compute (MvNC) SDK
* Movidius Neural Compute Application Zoo
* RGB Camera, e.g. RealSense D400 Series

## 3 Environment Setup
* Install ROS2 ([guide](https://github.com/ros2/ros2/wiki/Linux-Install-Debians))
* Create a ROS2 workspace
```Shell
mkdir -p ~/ros2_ws/src
```
* Install NCSDK [v1.12.00](https://github.com/movidius/ncsdk/releases) ([github](https://github.com/movidius/ncsdk))
* Install NC APP Zoo([github](https://github.com/movidius/ncappzoo))
* NCSDK should be installed in ```/opt/movidius``` by default. Create a symbol link in ```/opt/movidius``` to NC APP Zoo.
```Shell
sudo ln -s <your-workspace>/ncappzoo /opt/movidius/ncappzoo
```  
After that, make sure you can find graph data in ```/opt/movidius/ncappzoo/caffe``` or ```/opt/movidius/ncappzoo/tensorflow``` and image data in ```/opt/movidius/ncappzoo/data/images```
* Install object_msgs for ROS2 ([github](https://github.com/intel/ros2_object_msgs))
* Install ROS2 package for different cameras as needed. e.g.
  1. RealSense D400 series camera ([github](https://github.com/intel/ros2_intel_realsense))
   **Note**: Create a symbol link from libusb.a to libusb-1.0.a, otherwise "libusb.a" is probably not to be found by librealsense.  
    ```sudo ln -s /usr/lib/x86_64-linux-gnu/libusb-1.0.a /usr/lib/libusb.a```
* Install ROS2 Message Filters ([github](https://github.com/intel/ros2_message_filters))
```Shell
cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_message_filters
```
* Install ROS2 vision_opencv ([github](https://github.com/ros-perception/vision_opencv))
```Shell
cd ~/ros2_ws/src
git clone https://https://github.com/ros-perception/vision_opencv
cd vision_opencv
git checkout ros2
```
       
## 4 Building and Installation
```Shell
cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_intel_movidius_ncs
git clone https://github.com/intel/ros2_object_msgs
cd ~/ros2_ws
ament build
source install/local_setup.bash
```
Copy object label file from this project to NCSDK installation location.
```Shell
cp ~/ros2_ws/src/ros2_intel_movidius_ncs/data/labels/* /opt/movidius/ncappzoo/data/ilsvrc12/
```

## 5 Running the Demo
### 5.1 Classification
#### 5.1.1 Supported CNN Models
###### *Table1*
|CNN Model|Framework|Usage|
|:-|:-|:-|
|AlexNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#alexnet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#alexnet)|
|GoogLeNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#googlenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#googlenet)|
|SqueezeNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#squeezenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#squeezenet)|
|Inception_v1|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v1)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v1)|
|Inception_v2|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v2)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v2)|
|Inception_v3|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v3)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v3)|
|Inception_v4|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v4)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v4)|
|MobileNet|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#mobilenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#mobilenet)|
#### 5.1.2 Classification Result with GoogLeNet
![classification with googlenet](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/data/results/googlenet_dog.png "classification with googlenet")
#### 5.1.3 Running the Demo
* [Static Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md)
* [Video Streaming](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md)

### 5.2 Detection
#### 5.1.1 Supported CNN Models
|CNN Model|Framework|Usage|
|:-|:-|:-|
|MobileNetSSD(Recommended)|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md#mobilenet_ssd)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md#mobilenet_ssd)|
|TinyYolo_v1|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md#tinyyolo_v1)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md#tinyyolo_v1)|
#### 5.1.2 Detection Result with MobileNetSSD
![detection with mobilenetssd](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/data/results/mobilenetssd_car_bicycle.png "detection with mobilenetssd")
#### 5.1.3 Running the Demo
* [Static Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md)
* [Video Streaming](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md)

## 6 Interfaces
### 6.1 Topic
Classification: ```/movidius_ncs_nodelet/classified_objects```  
Detection: ```/movidius_ncs_nodelet/detected_objects```
### 6.2 Service
Classification: ```/movidius_ncs_image/classify_object```  
Detection: ```/movidius_ncs_image/detect_object```

## 7 Known Issues
* Only absolute path of image file supported in image inference demo
* Only test on RealSense D400 series camera

## 8 TODO
* Keep synchronized with [ROS NCS Package](https://github.com/intel/ros_intel_movidius_ncs/tree/master)


###### *Any security issue should be reported using process at https://01.org/security*
