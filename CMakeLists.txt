cmake_minimum_required(VERSION 3.5)
project(bumper2pc)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

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

add_executable(bumper2pc src/bumper2pc.cpp)
ament_target_dependencies(bumper2pc rclcpp std_msgs sensor_msgs)

set(msg_files
  "msg/Bumper.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  ${msg_files}
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_target_interfaces(bumper2pc ${PROJECT_NAME}_msgs rosidl_typesupport_cpp)


ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  bumper2pc
  DESTINATION lib/${PROJECT_NAME})

ament_package()
