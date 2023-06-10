cmake_minimum_required(VERSION 3.8)
project(proxy_holder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)
if (${uppercase_CMAKE_BUILD_TYPE} STREQUAL "RELEASE")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
    message(STATUS "Create Debug Build.")
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

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

# add_executable(proxy_holder_node
#   src/proxy_holder_node.cpp)

# ament_target_dependencies(proxy_holder_node
#   "rclcpp"
# )

# install(TARGETS
# proxy_holder_node
# DESTINATION lib/${PROJECT_NAME})

# add_library(${PROJECT_NAME} SHARED
#   src/proxy_holder_node.cpp)
# target_include_directories(${PROJECT_NAME} PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(${PROJECT_NAME}
#   PRIVATE "PROXY_HOLDER_CPP_BUILDING_DLL")
# ament_target_dependencies(${PROJECT_NAME}
#   "rclcpp"
#   "sensor_msgs"
#   "tf2"
# )


# # install(
# #   DIRECTORY lib/
# #   DESTINATION lib
# # )
# rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "proxy_holder::ProxyHolderNode" EXECUTABLE proxy_holder_node)
# install(
#   DIRECTORY src/
#   DESTINATION src
# )

# install(TARGETS
# ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ament_export_libraries(${PROJECT_NAME})

add_executable(proxy_holder_node 
  src/proxy_holder_node.cpp
)
ament_target_dependencies(proxy_holder_node rclcpp std_msgs tf2_msgs)

install(TARGETS
  proxy_holder_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()