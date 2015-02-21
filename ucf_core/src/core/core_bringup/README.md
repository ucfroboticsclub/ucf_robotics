# core_bringup
This package contains launch files to be used to bringup common nodes used in UCF workspaces.

#####Dependencies
<ul>
<li>usb_cam</li>
<li>hokuyo_node</li>
</ul>

## Table of Contents

[Launch Files](#launch)
  <ul>
  <li>[usb_cam.launch](#usb_cam)</li>
  <li>[hokuyo.launch](#hokuyo)</li>
  </ul>

## <a name="launch"></a>Launch Files
Below is a brief description and use information for the various launch files contained in this package.

##### <a name="usb_cam" />usb_cam.launch

This launch file launches a <a href="http://wiki.ros.org/usb_cam" >usb_cam_node</a> to communicate with a usb camera ( specifically tested with c920 ). As of now the launch file must be passed a camera name(cam_name) and a camera port(cam_port). ROS needs write/read access to the cam_port. UDEV rules can be added as indicated below in order to make this easier.

In a terminal:

```
sudo su
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", MODE="0666"' > /etc/udev/rules.d/99-uvc.rules 
exit
```

The launch file can be ran with following:

```
roslaunch core_bringup usb_cam.launch cam_name:=name_here cam_port:=/dev/videoX
```

To use the launch file within the vehicles minimal.launch add the following:

```
<include file="$(find core_bringup)/launch/usb_cam.launch>
  <arg name="cam_name" value="name_here" />
  <arg name="cam_port" value="/dev/videoX" />
</include>
```

##### <a name="hokuyo" />hokuyo.launch
This launch file launches a <a href="http://wiki.ros.org/hokuyo_node" >hokuyo_node</a> configured for 270 scan of a UTM lidar. Reference the <a href="http://wiki.ros.org/hokuyo_node" >wiki</a> for how to setup udev rules.

To use the launch file within the vehicles minimal.launch add the following:

```
<include file="$(find core_bringup)/launch/hokuyo.launch>
  <arg name="scan_topic" value="/scan" /> <!-- what topic should scan messages be published on -->
  <arg name="port" value="/dev/ttyACM0" />  <!-- tty port corresponding to hokuyo -->
</include>
```






