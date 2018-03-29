## Classification for Image
This project supports multiple CNN models for classification. Please make sure you have already [set up environment](https://github.com/intel/ros2_intel_movidius_ncs/tree/master#3-environment-setup) and [installed this project](https://github.com/intel/ros2_intel_movidius_ncs/tree/master#4-building-and-installation) correctly. You can refer to the following links for your interested models then.   
#### [1 CNN Models](#1-cnn-models-1)
* [AlexNet](#alexnet)
* [GoogleNet](#googlenet)
* [SqueezeNet](#squeezenet)
* [Inception_V1](#inception_v1)
* [Inception_V2](#inception_v2)
* [Inception_V3](#inception_v3)
* [Inception_V4](#inception_v4)
* [MobileNet](#mobilenet)
#### [2 Other Arguments](#2-other-arguments-1)
----------------------------------

### 1 CNN Models
* #### AlexNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/AlexNet
make
```
Launch object classification service.
```Shell
cd ~/ros2_ws
echo "param_file: alexnet.yaml" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_image_launch.py
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
* #### GoogleNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/GoogleNet
make
```
Launch object classification service.
```Shell
cd ~/ros2_ws
echo "param_file: googlenet.yaml" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_image_launch.py
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
* #### SqueezeNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/SqueezeNet
make
```
Launch object classification service.
```Shell
cd ~/ros2_ws
echo "param_file: squeezenet.yaml" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_image_launch.py
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
* #### Inception_V1
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v1
make
```
Launch object classification service.
```Shell
cd ~/ros2_ws
echo "param_file: inception_v1.yaml" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_image_launch.py
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
* #### Inception_V2
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v2
make
```
Launch object classification service.
```Shell
cd ~/ros2_ws
echo "param_file: inception_v2.yaml" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_image_launch.py
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
* #### Inception_V3
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v3
make
```
Launch object classification service.
```Shell
cd ~/ros2_ws
echo "param_file: inception_v3.yaml" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_image_launch.py
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
* #### Inception_V4
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v4
make
```
Launch object classification service.
```Shell
cd ~/ros2_ws
echo "param_file: inception_v4.yaml" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_image_launch.py
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
* #### MobileNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/mobilenets
make
```
Launch object classification service.
```Shell
cd ~/ros2_ws
echo "param_file: mobilenet.yaml" > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml
ament build
source install/local_setup.bash
launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_image_launch.py
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
ros2 run movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
### 2 Other Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|device_index|ncs device index|0|0~N-1(N is the maximum number of inserted NCS devices)|
|log_level|ncs log level|0|0:Nothing / 1:Errors / 2:Verbose|
|cnn_type|indicate different cnn models|googlenet|alexnet / googlenet / squeezenet / inception_v1 / inception_v2 / inception_v3 / inception_v4 / mobilenet|
|top_n|the number of results to be shown, only valid for classification|3|0~5|
