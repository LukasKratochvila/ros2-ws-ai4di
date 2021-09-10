cmake_minimum_required(VERSION 3.5)
project(cpp_examples)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)


include_directories(include ${custom_interfaces_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)
target_include_directories(publisher PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
target_include_directories(subscriber PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(service src/service.cpp)
target_include_directories(service PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(service rclcpp example_interfaces)

add_executable(client src/client.cpp)
target_include_directories(client PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(client rclcpp example_interfaces)

add_executable(action_server src/action_server.cpp)
target_include_directories(action_server PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_server rclcpp rclcpp_action rclcpp_components custom_interfaces)


add_executable(action_client src/action_client.cpp)
target_include_directories(action_client PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_client rclcpp rclcpp_action rclcpp_components custom_interfaces)

install(TARGETS publisher subscriber service client action_server action_client DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
