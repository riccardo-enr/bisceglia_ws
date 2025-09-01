# px4-progetto-tesi
sudo apt update sudo apt install -y ros-humble-px4-msgs ros-humble-ros-gz-bridge

BUILD

mkdir -p ~/mocap_ws/src cd ~/mocap_ws/src git clone https://github.com//.git cd ~/mocap_ws source /opt/ros/humble/setup.bash colcon build --symlink-install source install/setup.bash

ESECUZIONE

MicroXRCEAgent udp4 -p 8888

cd ~/PX4-Autopilot make px4_sitl gz_x500

ros2 launch mocap_bridge mocap_bridge.launch.py
