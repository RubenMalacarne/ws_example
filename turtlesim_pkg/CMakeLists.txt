cmake_minimum_required(VERSION 3.8)
project(turtlesim_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces_pkg REQUIRED)

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

add_executable(turtle_controller_node src/turtle_controller_node.cpp)
ament_target_dependencies(turtle_controller_node rclcpp std_msgs turtlesim geometry_msgs interfaces_pkg)

add_executable(turtle_spawn_node src/turtle_spawn_node.cpp)
ament_target_dependencies(turtle_spawn_node rclcpp std_msgs turtlesim interfaces_pkg)


install(TARGETS
  turtle_controller_node
  turtle_spawn_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
