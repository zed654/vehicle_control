
	< include >

		-> can_linux_val.h
			: It is just code that are made to adjust form to PEAKCAN Driver

		-> coord_map.h
			: To operate coord_map.cpp, related code is writen in here.

		-> env_setting.h
			: It include environment code like CAN env setting, CAN enable, GNSS enable, ...

		-> parser_vercpp.hpp
			: To operate GPS, related code is writen in here.
			: It is not code made by me

		-> PCBUSB.h
			: It is used when operating CAN R/W by using PEAKCAN Driver
			: But in Ubuntu, We use SocketCAN. So It has not been used.
			: I added it to operate CAN R/W on MacOS

		-> pd_control.h
			: It is used Longitude control

		-> ROS_header.hpp
			: To operate ROS, related code is writen in here.

		-> TSR_receive.h
			: To operate TSR from ROS, related code is writen in here.



	< src >

		-> ioniq_control_node.cpp
			: It include main loop
			: hyper param, such as APM enable, ASM enable, Opencv_View_map Size, PID gain, ..., can be modified in here
			: pthread initialized in here. (CAN_RW, dead_reckoning, GNSS_receive, path_follow, ROS_enable)
			: main while loop show the map


		-> CAN_RW__thread__.cpp
			: CAN data Read / Write thread
			: This includes Chassis CAN and Control CAN data

		-> coord_map.cpp
			: It take a saved coordinate map data (Setpoint, heading, Target speed, ... )
			: It draw a map with x and y increasing by 4 times.

		-> dead_reckoning__thread__.cpp
			: It calculate position of vehicle between DGPS received data by using yaw rate and vehicle speed, extracted from CAN

		-> GNSS_receive__thread__.cpp
			: It bring position data by using DGPS that is attached on vehicle. (Frequency : 5Hz)

		-> parser_vercpp.cpp
			: It extract position of x, y, thetha from data of DGPS.
			: It's not my code. made by other person.

		-> path_follow__thread.cpp
			: longitudinal, lateral control code is included.
			: additionally, The method of changing target waypoint is included

		-> ROS_enable__thread__.cpp
			: To get Traffic Sign Recognition result, I made it.
			: It bring TSR result calculated YOLO V3 from darknet_ros_msgs.
