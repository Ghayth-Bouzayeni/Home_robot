#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Wait for MQTT to be ready
echo "Waiting for MQTT broker..."
while ! nc -z mqtt 1883; do
  sleep 0.1
done
echo "MQTT broker is ready"

# Launch ROS 2 nodes
echo "Launching ROS 2 nodes..."
exec ros2 launch robot_simulation house_sim.launch.py &
sleep 5
ros2 launch robot_simulation house_slam.launch.py &
sleep 5
ros2 launch robot_simulation autonomous_navigation.launch.py &

# Keep container running
wait