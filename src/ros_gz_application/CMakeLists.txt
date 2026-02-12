cmake_minimum_required(VERSION 3.5)

project(ros_gz_application)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
	scripts/cmd_vel_publisher.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
