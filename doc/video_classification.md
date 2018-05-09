## Classification for Video
This project supports multiple CNN models for classification. Please make sure you have already [set up environment](https://github.com/intel/ros2_intel_movidius_ncs/tree/master#3-environment-setup) and [installed this project](https://github.com/intel/ros2_intel_movidius_ncs/tree/master#4-building-and-installation) correctly. You can refer to the following links for your interested models then.  
#### [1 CNN Models](#1-cnn-models-1)
* [AlexNet](#alexnet)
* [GoogLeNet](#googlenet)
* [SqueezeNet](#squeezenet)
* [Inception_V1](#inception_v1)
* [Inception_V2](#inception_v2)
* [Inception_V3](#inception_v3)
* [Inception_V4](#inception_v4)
* [MobileNet](#mobilenet)
#### [2 Run with Other ROS Supported Cameras](#2-run-with-other-ros-supported-cameras-1)
#### [3 Other Arguments](#3-other-arguments-1)
----------------------------------

### 1 CNN Models
* #### AlexNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/AlexNet
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: alexnet.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
echo -e "param_file: alexnet.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
ros2 run movidius_ncs_example movidius_ncs_example_stream_classification
```
* #### GoogLeNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/GoogLeNet
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: googlenet.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
echo -e "param_file: googlenet.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
ros2 run movidius_ncs_example movidius_ncs_example_stream_classification
```
* #### SqueezeNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/SqueezeNet
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: squeezenet.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
echo -e "param_file: squeezenet.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
ros2 run movidius_ncs_example movidius_ncs_example_stream_classification
```
* #### Inception_V1
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v1
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: inception_v1.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
echo -e "param_file: inception_v1.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
ros2 run movidius_ncs_example movidius_ncs_example_stream_classification
```
* #### Inception_V2
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v2
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: inception_v2.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
echo -e "param_file: inception_v2.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
ros2 run movidius_ncs_example movidius_ncs_example_stream_classification
```
* #### Inception_V3
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v3
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: inception_v3.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
echo -e "param_file: inception_v3.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
ros2 run movidius_ncs_example movidius_ncs_example_stream_classification
```
* #### Inception_V4
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v4
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: inception_v4.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
echo -e "param_file: inception_v4.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
ros2 run movidius_ncs_example movidius_ncs_example_stream_classification
```
* #### MobileNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/mobilenets
make
```
Launch video streaming nodelet.
- Option 1: RealSense
```Shell
# Choose CNN model
cd ~/ros2_ws
echo -e "param_file: mobilenet.yaml\ninput_topic: /camera/color/image_raw" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
echo -e "param_file: mobilenet.yaml\ninput_topic: /image" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
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
ros2 run movidius_ncs_example movidius_ncs_example_stream_classification
```
### 2 Run with Other ROS Supported Cameras
TODO

### 3 Other Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|device_index|ncs device index|0|0~N-1(N is the maximum number of inserted NCS devices)|
|log_level|ncs log level|0|0:Nothing / 1:Errors / 2:Verbose|
|cnn_type|indicate different cnn models|googlenet|alexnet / googlenet / squeezenet / inception_v1 / inception_v2 / inception_v3 / inception_v4 / mobilenet|
|top_n|the number of results to be shown, only valid for classification|3|0~5|
