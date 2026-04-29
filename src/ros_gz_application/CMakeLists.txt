cmake_minimum_required(VERSION 3.5)

project(ros_gz_application)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
	launch/cmd_vel_publisher.py
	DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
	noder/lidar_sweeper.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME lidar_sweeper
)

install(PROGRAMS
	noder/laser_assembler.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME laser_assembler
)

install(PROGRAMS
	noder/YOLO.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME YOLO
	)


ament_package()
