cmake_minimum_required(VERSION 3.5)

project(ros_gz_application)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
	launch/cmd_vel_publisher.py
	DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
	nodes/lidar_sweeper.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME lidar_sweeper
)

install(PROGRAMS
	nodes/laser_assembler.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME laser_assembler
)

install(PROGRAMS
	nodes/lidar_settle_calibrator.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME lidar_settle_calibrator
)

install(PROGRAMS
	nodes/YOLO.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME YOLO
)

install(PROGRAMS
	nodes/YOLO_gstreamer.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME YOLO_gstreamer
)

install(PROGRAMS
	nodes/IMU.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME IMU	
)

install(PROGRAMS
	nodes/allocator.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME allocator
)

install(PROGRAMS
	nodes/thermal_converter.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME thermal_converter
)

install(PROGRAMS
	nodes/thermal_processing.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME thermal_processing
)

install(PROGRAMS
	nodes/ir_sensor_driver.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME ir_sensor_driver
)

install(PROGRAMS
	nodes/collision_avoidance.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME collision_avoidance
)

ament_package()
