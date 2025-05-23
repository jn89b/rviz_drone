cmake_minimum_required(VERSION 3.5)

#change name of package to this
project(rviz_drone)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Python-Documentation.html

# Find dependencies this will allow CPP and python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Telem.msg"
#   "msg/CtlTraj.msg"

#   "srv/AddTwoInts.srv"
#   "srv/SetArmDisarm.srv"
#   "srv/SetGoal.srv"
#   "srv/SetUASType.srv"
#   "srv/SetTakeOff.srv"
#   # "srv/getGSInfo.srv"
#  )
# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
add_executable(aircraft_frame_node src/aircraft_frame_node.cpp src/aircraft_frame_c.cpp)
ament_target_dependencies(
  aircraft_frame_node
    rclcpp
    std_msgs
    drone_interfaces
    tf2
    tf2_ros
)
# Install Cpp executables
install(TARGETS
  aircraft_frame_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/aircraft_vis.py
  scripts/aircraft_frame.py
  scripts/traj_vis.py
  scripts/obstacle_vis.py
  scripts/goal_vis.py
  scripts/aircraft_actual_frame.py
  # scripts/directional_traj.py
  # scripts/avoid_traj.py
  # scripts/waypoint_traj.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

##tests 
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    # tests/testDroneCommander.py
    # Add other test files here
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

#launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)