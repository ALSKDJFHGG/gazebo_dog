cmake_minimum_required(VERSION 3.8)
project(custom_nav_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(custom_nav_planners SHARED
  src/custom_global_planner.cpp
  src/custom_local_planner.cpp
)

ament_target_dependencies(custom_nav_planners
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS custom_nav_planners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

install(FILES
  plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

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
