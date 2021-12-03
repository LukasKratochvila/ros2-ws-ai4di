cmake_minimum_required(VERSION 3.5)
project(pcl_pub_cpp)

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
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pcl_pub_cpp_node src/pcl_pub_cpp_node.cpp)
target_include_directories(pcl_pub_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pcl_pub_cpp_node 
	stdc++fs
	${PCL_LIBRARIES})
ament_target_dependencies(pcl_pub_cpp_node 
	cv_bridge 
	rclcpp 
	sensor_msgs
	pcl_conversions)

install(TARGETS pcl_pub_cpp_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
