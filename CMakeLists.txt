cmake_minimum_required(VERSION 3.8)
project(elevator_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(capella_ros_msg REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_common   REQUIRED)
find_package(nav2_msgs REQUIRED)

set(dependencies 
  angles
  geometry_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_geometry_msgs
  tf2_ros
  capella_ros_msg
  aruco_msgs
  ament_cmake
  nav2_common
  nav2_msgs
)

# drive_into_elevator_lib
add_library(drive_into_elevator_lib  SHARED)
target_sources(drive_into_elevator_lib 
  PRIVATE 
  src/drive_into_elevator_action.cpp)
target_include_directories(drive_into_elevator_lib PUBLIC include)
ament_target_dependencies(drive_into_elevator_lib ${dependencies})
rclcpp_components_register_node(drive_into_elevator_lib
    PLUGIN "elevator_pkg::DriveIntoElevatorActionServer"
    EXECUTABLE drive_into_elevator_action_server)

# drive_outof_elevator_lib
add_library(drive_outof_elevator_lib  SHARED)
target_sources(drive_outof_elevator_lib 
  PRIVATE 
  src/drive_outof_elevator_action.cpp)
target_include_directories(drive_outof_elevator_lib PUBLIC include)
ament_target_dependencies(drive_outof_elevator_lib ${dependencies})
rclcpp_components_register_node(drive_outof_elevator_lib
    PLUGIN "elevator_pkg::DriveOutofElevatorActionServer"
    EXECUTABLE drive_outof_elevator_action_server)

# move_to_goal_lib
add_library(move_to_goal_lib  SHARED)
target_sources(move_to_goal_lib 
  PRIVATE 
  src/move_to_goal_action.cpp)
target_include_directories(move_to_goal_lib PUBLIC include)
ament_target_dependencies(move_to_goal_lib ${dependencies})
rclcpp_components_register_node(move_to_goal_lib
    PLUGIN "elevator_pkg::MoveToGoalActionServer"
    EXECUTABLE move_to_goal_action_server)

set(libraries_names
drive_into_elevator_lib
drive_outof_elevator_lib
move_to_goal_lib
)
  install(TARGETS ${libraries_names}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  install(DIRECTORY include/
    DESTINATION include
  )
  
  install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
  )
  
  ament_export_include_directories(include)
  ament_export_libraries(${libraries_names})
  ament_export_dependencies(${dependencies})
  

ament_package()
