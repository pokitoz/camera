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


By default, WSL2 is using NAT, meaning that it will have its own IP address different from Windows. It means that your RPi and WSL2 are not on the same subnet and cannot communicate through multicast by default. The NAT will block some of the traffic and the discovery won't work. The DDS protocol to send packets in ROS2 needs multicast or you need to update it to unicast (specifying directly the IP of the node).

Using multicast

The "Mirrored Networking" mode explained here would enable this support:

Enabling this changes WSL to an entirely new networking architecture which has the goal of 'mirroring' the network interfaces that you have on Windows into Linux, to add new networking features and improve compatibility.

[...]

Here are the current benefits to enabling this mode:

[...]

Multicast support

Basically, WSL and Windows would have the same IP address.

It implies creating/updating a .wslconfig file in Windows: %UserProfile%\.wslconfig with the following content:

[wsl2]
networkingMode=mirrored

To enable inbound packets, run the following in Powershell:
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow

Make sure to restart WSL completely or reboot and you should have everything enabled.

ifconfig and ipconfig should show the same inet address.



Testing using Jazzy

On the RPi, do the following (note the --net=host to make sure everything is forwarded as docker will share the host’s network stack directly (no virtual interfaces, no NAT, no port forwarding.) see doc)

docker run --net=host --rm ros:jazzy-ros-core ros2 topic pub mytopic std_msgs/msg/Int32 "data: 128"

On WSL, do the following

ros2 topic echo mytopic std_msgs/msg/Int32
