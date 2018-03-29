## Detection for Video
This project supports multiple CNN models for detection. Please make sure you have already [set up environment](https://github.intel.com/intel/ros2_intel_movidius_ncs/tree/master#3-environment-setup) and [installed this project](https://github.intel.com/intel/ros2_intel_movidius_ncs/tree/master#4-building-and-installation) correctly. You can refer to the following links for your interested models then.  
#### [1 CNN Models](#1-cnn-models-1)
* [MobileNet_SSD](#mobilenet_ssd)
* [TinyYolo_V1](#tinyyolo_v1)
#### [2 Run with Other ROS Supported Cameras](#2-run-with-other-ros-supported-cameras-1)
#### [3 Other Arguments](#3-other-arguments-1)
----------------------------------

### 1 CNN Models
* #### MobileNet_SSD
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/SSD_MobileNet
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: mobilenetssd.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
# Terminal 1, run realsense camera
realsense_ros2_camera
# Terminal 2, run video streaming composition manager
ros2 run composition api_composition
# Terminal 3, launch video streaming composistion
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_stream_launch.py
```
- Option 2: Astra
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: mobilenetssd.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
# Terminal 1, run astra camera
ros2 run astra_camera astra_camera_node -w 640 -h 480 -I -D
# Terminal 2, run video streaming composition manager
ros2 run composition api_composition
# Terminal 3, launch video streaming composistion
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_stream_launch.py
```
Launch image viewer to show the classification result on another console.
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_stream_detection
```
* #### TinyYolo_V1
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/TinyYolo
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: tinyyolo_v1.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
# Terminal 1, run realsense camera
realsense_ros2_camera
# Terminal 2, run video streaming composition manager
ros2 run composition api_composition
# Terminal 3, launch video streaming composistion
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_stream_launch.py
```
- Option 2: Astra
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: tinyyolo_v1.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
# Terminal 1, run astra camera
ros2 run astra_camera astra_camera_node -w 640 -h 480 -I -D
# Terminal 2, run video streaming composition manager
ros2 run composition api_composition
# Terminal 3, launch video streaming composistion
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_stream_launch.py
```
Launch image viewer to show the classification result on another console.
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_stream_detection
```
### 2 Run with Other ROS Supported Cameras
TODO
### 3 Other Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|device_index|ncs device index|0|0~N-1(N is the maximum number of inserted NCS devices)|
|log_level|ncs log level|0|0:Nothing / 1:Errors / 2:Verbose|
|cnn_type|indicate different cnn models|googlenet|mobilenetssd / tinyyolo_v1|
