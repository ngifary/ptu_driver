cmake_minimum_required(VERSION 3.8)
project(ptu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

set(node_dependencies
  rclcpp
  std_msgs
  sensor_msgs
)

## Declare a cpp library
add_library(PCA9685 SHARED
  src/PCA9685.c
)
ament_target_dependencies(PCA9685)
ament_export_libraries(PCA9685)

## Declare cpp executables
add_executable(ptu_node src/ptu_node.cpp)
target_link_libraries(ptu_node PCA9685)
ament_target_dependencies(ptu_node ${node_dependencies})

install(TARGETS
  PCA9685
  ptu_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
