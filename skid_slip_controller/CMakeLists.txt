cmake_minimum_required(VERSION 3.8)
project(skid_slip_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Library: Core components
add_library(${PROJECT_NAME}_core SHARED
  src/turn_radius_constraint.cpp
  src/slip_detector.cpp
  src/surface_classifier.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# Executable: Adaptive controller node
add_executable(adaptive_controller_node
  src/adaptive_controller_node.cpp
)

target_link_libraries(adaptive_controller_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(adaptive_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  adaptive_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(test_turn_radius_constraint
    test/test_turn_radius_constraint.cpp
  )
  target_link_libraries(test_turn_radius_constraint
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_slip_detector
    test/test_slip_detector.cpp
  )
  target_link_libraries(test_slip_detector
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_surface_classifier
    test/test_surface_classifier.cpp
  )
  target_link_libraries(test_surface_classifier
    ${PROJECT_NAME}_core
  )
endif()

ament_package()
