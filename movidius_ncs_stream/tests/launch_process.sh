gnome-terminal -x bash -c "realsense_ros2_camera"

gnome-terminal -x bash -c "api_composition"

gnome-terminal -x bash -c "launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_stream_launch.py"
