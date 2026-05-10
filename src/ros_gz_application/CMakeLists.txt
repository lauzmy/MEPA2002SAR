cmake_minimum_required(VERSION 3.5)

project(ros_gz_application)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
	launch/cmd_vel_publisher.py
	DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
	nodes/lidar3d.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME lidar3d
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
	nodes/thermal_reading.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME thermal_reading
)

install(PROGRAMS
	nodes/thermal_processing.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME thermal_processing
)

install(PROGRAMS
	nodes/collision_avoidance.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME collision_avoidance
)

install(PROGRAMS
	nodes/bamse_locator.py
	DESTINATION lib/${PROJECT_NAME}
	RENAME bamse_locator
)

install(PROGRAMS
    nodes/exploration_monitor.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME exploration_monitor
)

ament_package()
