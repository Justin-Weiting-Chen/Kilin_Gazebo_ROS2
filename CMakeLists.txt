cmake_minimum_required(VERSION 3.8)
project(kilin_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz_common REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED) 
find_package(controller_manager REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(hardware_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(Kilin_Coordinator src/Kilin_Coordinator.cpp)

ament_target_dependencies(Kilin_Coordinator
  rclcpp
  trajectory_msgs
  controller_manager_msgs
  std_msgs
  sensor_msgs
  robot_state_publisher
  ros2_control
)

# 確保編譯器能找到頭文件
target_include_directories(Kilin_Coordinator PRIVATE include)

# 確保目標文件能被 ROS 2 找到並執行
install(TARGETS Kilin_Coordinator
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  trajectory_msgs
  controller_manager_msgs
  std_msgs
  sensor_msgs
)


install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
