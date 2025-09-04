# px4-progetto-tesi

## Step 1 - pacchetti

 - Installare i pacchetti PX4 message e Gazebo bridge


## Step 2 - Builda _mocap_bridge_

- Crea ROS2 workspace e clona repository

  `mkdir -p ~/mocap_ws/src`

  `cd ~/mocap_ws/src`

  git clone https://github.com//.git

  cd ~/mocap_ws

- Source ROS2 Humble and builda el workspace

  source /opt/ros/humble/setup.bash

  colcon build --symlink-install

  source install/setup.bash

## Step 3 - ESECUZIONE

MicroXRCEAgent udp4 -p 8888

cd ~/PX4-Autopilot

make px4_sitl gz_x500

ros2 launch mocap_bridge mocap_bridge.launch.py

| Parameter      | Changed value           | Default value           |
| -------------- | ----------------------- | ----------------------- |
| EKF2_BARO_CTRL | 0 (Disabled)            | 1 (Enabled)             |
| EKF2_EV_CTRL   | 3 (Horizontal+Vertical) | 15 (All flags selected) |
| EKF2_EV_DELAY  | 10.0 ms                 | 0.0 ms                  |
| EKF2_HGT_REF   | 3 (Vision)              | 1 (GPS)                 |
