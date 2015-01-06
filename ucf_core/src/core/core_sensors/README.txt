This package contains source and launch files for sensors used by ucf robots. Below is usage information



---------------------------------------------------------
Camera -- USB camera ( logitech Cameras )

1. Setup udev rules

	Run:

		sudo echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", MODE="0666"' >> /etc/udev/rules.d/99-uvc.rules

2. Get Serials for camera

	Run: (replace '/dev/video0' with correct reference to webcam)

		udevadm info -a -p $(udevadm info -q path -n /dev/video0)

	Look for:

		ATTRS{serial}=="<some serial here>"

		This is the serial number for the camera to be used in launch files

	
3. Launch file example:

	----------------------------------------------------------------------------------
		<launch>

			<include file="$(find core_sensors)/launch/usb_camera_stream.launch" >
				<arg name="cam_name" value="<camera name here>" />
				<arg name="cam_serial" value="<serial from above here>" />
			</include>

		</launch>

  ----------------------------------------------------------------------------------

------------------------------------------------------------------------