cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/CustomMsg.msg"
        "srv/CustomService.srv"
        "action/CustomAction.action"
        )

ament_export_dependencies(rosidl_default_runtime)
ament_package()