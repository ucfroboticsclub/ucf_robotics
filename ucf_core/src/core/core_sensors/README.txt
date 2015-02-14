	
3. Launch file example:

	----------------------------------------------------------------------------------
		<launch>

			<include file="$(find core_sensors)/launch/usb_cam.launch" >
				<arg name="cam_name" value="<camera name here>" />
				<arg name="cam_port" value="/dev/videoX" />
			</include>

		</launch>

  ----------------------------------------------------------------------------------

------------------------------------------------------------------------