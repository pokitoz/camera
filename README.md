## ESP32

- Open with VSCode and launch in docker (.devcontainer)
- Source esp32 setup (see Dockerfile)
- Use `idf.py build`
- Connect a FTDI, use 3v3 and make sure to connect the 2 IOs to flash
- Reset the device with the button
- Flash the device
- Monitor
- Remove the wire and reboot

## RPi or WSL

- See Discovery section, the microAgent needs to be enabled.
- Source ROS2 setup environment and use the python script to save the 
  stream from the topic `img`.

## Discovery

microROS is configured to use dynamic discovery to the micro Agent as follow:
```
https://micro-xrce-dds.docs.eprosima.com/en/latest/
cd ~/Micro-XRCE-DDS-Agent
./MicroXRCEAgent udp4 -p 8888 -d
```

It would display the following message:
```
[1744220938.727789] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1744220938.731001] info     | DiscoveryServerLinux.cpp | init                     | running...             | Port: 7400
[1744220938.731465] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4

[1744221262.291822] info     | Root.cpp           | create_client            | create                 | client_key: 0x5ABEF705, session_id: 0x81
[1744221262.291949] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x5ABEF705, address: 192.168.129.16:43200
[1744221262.320607] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x5ABEF705, participant_id: 0x000(1)
[1744221262.341044] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x5ABEF705, topic_id: 0x000(2), participant_id: 0x000(1)
[1744221262.351388] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x5ABEF705, publisher_id: 0x000(3), participant_id: 0x000(1)
[1744221262.376011] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x5ABEF705, datawriter_id: 0x000(5), publisher_id: 0x000(3)
```


It can be setup on WSL2 or a RPi (See https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html).
Using Docker is an easy way to setup everything:
```
docker pull ros:jazzy-ros-core
docker run -it --rm ros:jazzy-ros-core
```

There are other way to use discovery, like:
- https://docs.ros.org/en/rolling/Tutorials/Advanced/Improved-Dynamic-Discovery.html


By default, WSL2 is using NAT, meaning that it will have its own IP address
different from Windows. It means that your RPi and WSL2 are not on the same
subnet and cannot communicate through multicast by default.
The NAT will block some of the traffic and the discovery won't work.
The DDS protocol to send packets in ROS2 needs multicast or you need to
update it to unicast (specifying directly the IP of the node).

### Using multicast

The "Mirrored Networking" mode explained here would enable this support:

> Enabling this changes WSL to an entirely new networking architecture which
> has the goal of 'mirroring' the network interfaces that you have on Windows
> into Linux, to add new networking features and improve compatibility.$
> [...]
> Here are the current benefits to enabling this mode:
> [...]
> - Multicast support

Basically, WSL and Windows would have the same IP address.

It implies creating/updating a .wslconfig file in Windows: `%UserProfile%\.wslconfig`
with the following content:

```
[wsl2]
networkingMode=mirrored
```

To enable inbound packets, run the following in Powershell:

`Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow`

Make sure to restart WSL completely or reboot and you should have everything enabled.
`ifconfig` and `ipconfig` should show the same inet address.


### Testing using Jazzy

On the RPi or WSL, do the following (note the `--net=host` to make sure everything
is forwarded as docker will share the host’s network stack directly
(no virtual interfaces, no NAT, no port forwarding.) see doc)

```
docker run --net=host --rm ros:jazzy-ros-core ros2 topic pub mytopic std_msgs/msg/Int32 "data: 128"
```

On WSL or RPi, do the following:

```
ros2 topic echo mytopic std_msgs/msg/Int32
```