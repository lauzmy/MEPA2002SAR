cmake_minimum_required(VERSION 3.5)

project(gregor_bringup)

find_package(ament_cmake REQUIRED)
find_package(gregor_sim_description REQUIRED)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

# USE_SOURCE_PERMISSIONS preserves the executable bit on mola-map.sh.
install(
  DIRECTORY
    scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
