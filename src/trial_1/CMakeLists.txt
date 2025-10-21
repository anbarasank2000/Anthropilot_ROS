cmake_minimum_required(VERSION 3.8)
project(trial_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_srvs REQUIRED)

# sources for recovery plugin
include_directories(include)

add_library(trial_1_recovery SHARED
  src/spin_recovery.cpp
)

ament_target_dependencies(trial_1_recovery
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  pluginlib
  nav2_core
  nav2_util
  std_srvs
)

target_compile_features(trial_1_recovery PUBLIC c_std_11 cxx_std_17)

install(TARGETS trial_1_recovery
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
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

install(DIRECTORY
  launch
  map
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)
