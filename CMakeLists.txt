cmake_minimum_required(VERSION 3.5)
project(turtle_control_cpp)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(error_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# executable
add_executable(draw_circle src/draw_circle.cpp)
add_executable(draw_fb_circle src/draw_fb_circle.cpp)
add_executable(draw_fb_eight src/draw_fb_eight.cpp)
add_executable(draw_pid_circle src/draw_pid_circle.cpp)


# dependencies
ament_target_dependencies(draw_circle rclcpp std_msgs geometry_msgs std_srvs turtlesim)
ament_target_dependencies(draw_fb_circle rclcpp std_msgs geometry_msgs std_srvs turtlesim error_msgs)
ament_target_dependencies(draw_fb_eight rclcpp std_msgs geometry_msgs std_srvs turtlesim error_msgs)
ament_target_dependencies(draw_pid_circle rclcpp std_msgs geometry_msgs std_srvs turtlesim error_msgs)

# install
install(TARGETS
  draw_circle
  draw_fb_circle
  draw_fb_eight
  draw_pid_circle
  DESTINATION lib/${PROJECT_NAME})
