cmake_minimum_required(VERSION 3.5)
project(proxy_holder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)


set(dependencies
  rclcpp
  std_msgs
  tf2_msgs
  tf2_geometry_msgs
)

set(SOURCES
  src/PID.cpp
  src/serialib.cpp
  src/CrsfSerial.cpp
)

# Include our own headers
include_directories(include)

add_library(my_library SHARED ${SOURCES})
ament_target_dependencies(my_library 
  ${dependencies}
)

rclcpp_components_register_nodes(my_library
  PLUGIN "ProxyHolderNode"
  EXECUTABLE proxy_holder_node
)

# Create a node
add_executable(my_node src/main.cpp)
target_link_libraries(my_node my_library)

ament_target_dependencies(my_node
  ${dependencies}
)


# Install our headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install our node and library
install(TARGETS my_library my_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Tell downstream packages where to find our headers
ament_export_include_directories(include)
# Tell downstream packages our libraries to link against
ament_export_libraries(my_library)
# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  ${dependencies}
)
ament_package()