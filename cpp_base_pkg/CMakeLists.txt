cmake_minimum_required(VERSION 3.8)
project(cpp_base_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
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


add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(NumberPublisherNode src/num_publisher_node.cpp)
ament_target_dependencies(NumberPublisherNode rclcpp example_interfaces)

add_executable(NumberCounterNode src/num_counter_node.cpp)
ament_target_dependencies(NumberCounterNode rclcpp example_interfaces)

add_executable(add_two_ints_client_node src/add_two_ints_client_node.cpp)
ament_target_dependencies(add_two_ints_client_node rclcpp example_interfaces)

add_executable(add_two_ints_server_node src/add_two_ints_server_node.cpp)
ament_target_dependencies(add_two_ints_server_node rclcpp example_interfaces)

add_executable(hw_status_publisher_node src/hw_status_publisher_node.cpp)
ament_target_dependencies(hw_status_publisher_node rclcpp interfaces_pkg)

install(TARGETS
  cpp_node
  NumberPublisherNode
  NumberCounterNode
  add_two_ints_server_node
  add_two_ints_client_node
  hw_status_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
