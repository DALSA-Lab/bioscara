cmake_minimum_required(VERSION 3.8)
project(moveit_motion_plans)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(pilz_sequence src/pilz_sequence.cpp)
target_include_directories(pilz_sequence PUBLIC include)

ament_target_dependencies(
  pilz_sequence # Target
  rclcpp
  rclcpp_action
  moveit_visual_tools
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS
    pilz_sequence
  DESTINATION
    lib/${PROJECT_NAME}
)

# TODO: find a way to not add every script to the cmakelist
install(PROGRAMS
        scripts/IAmNotARobot.py
        scripts/PickPlace.py
        scripts/PickPlace_continous.py
        scripts/PickPlace_markers.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
        launch
        #config
        DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights REQUIRED)
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
