- Need some basic packets on Windows like cmake and make
- or use Docker and .devcontainer

export ROS_STATIC_PEERS='192.168.129.16;192.168.129.15;'
https://docs.ros.org/en/rolling/Tutorials/Advanced/Improved-Dynamic-Discovery.html

https://micro-xrce-dds.docs.eprosima.com/en/latest/
cd ~/Micro-XRCE-DDS-Agent
./MicroXRCEAgen udp4 -p 8888 -d

https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html
docker pull ros:jazzy-ros-core
docker run -it --rm ros:jazzy-ros-core

cd /opt/ros/jazzy/bin
./ros2 topic echo freertos_int32_publisher std_msgs/msg/Int32