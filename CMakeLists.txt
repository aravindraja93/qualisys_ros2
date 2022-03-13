cmake_minimum_required(VERSION 3.5)
project(qualisys_ros2)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_library(qualisys
    src/Markup.cpp
    src/Network.cpp
    src/RTPacket.cpp
    src/RTProtocol.cpp)

add_executable(qualisys_node src/qualisys_node.cpp)

target_link_libraries(qualisys_node qualisys)

ament_target_dependencies(qualisys_node tf2_ros rclcpp rclcpp_action nav_msgs geometry_msgs)

install(TARGETS
  qualisys_node
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/image_combine.py
  scripts/mocap_gazebo_sync.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
