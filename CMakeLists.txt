cmake_minimum_required(VERSION 3.8)
project(abb_highlevel_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(abb_robot_msgs REQUIRED) 

find_package(moveit_ros_planning_interface REQUIRED) # Core MoveIt C++ API
find_package(geometry_msgs REQUIRED)                 # For Pose and other message types
find_package(tf2_geometry_msgs REQUIRED)   
find_package(moveit_core REQUIRED) 

# Executable for end effector control
add_executable(abb_endeffector_control_client src/endEffector.cpp)
ament_target_dependencies(abb_endeffector_control_client rclcpp abb_robot_msgs) 

install(TARGETS abb_endeffector_control_client
  DESTINATION lib/${PROJECT_NAME}
)


# Executable for inverse kinematics (The one causing the linker error)
add_executable(inverse_kinematics src/inverse_kinematics.cpp)

# FIX: Ensure all dependent libraries are explicitly listed here.
# moveit_ros_planning_interface provides the definition for execute().
ament_target_dependencies(
  inverse_kinematics 
  rclcpp 
  moveit_ros_planning_interface 
  moveit_core
  geometry_msgs  
  tf2_geometry_msgs
)

# FIX: Install the executable so it can be run
install(TARGETS inverse_kinematics
  DESTINATION lib/${PROJECT_NAME}
)


# Executable for inverse kinematics (The one causing the linker error)
add_executable(semantic_publisher src/semantic_publisher.cpp)

# FIX: Ensure all dependent libraries are explicitly listed here.
# moveit_ros_planning_interface provides the definition for execute().
ament_target_dependencies(
    semantic_publisher
    rclcpp
    moveit_ros_planning_interface
    moveit_core
    geometry_msgs
    tf2_geometry_msgs
)
# FIX: Install the executable so it can be run
install(TARGETS semantic_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
