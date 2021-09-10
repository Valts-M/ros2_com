cmake_minimum_required(VERSION 3.8)
project(ros2_com)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set (CMAKE_CXX_STANDARD_REQUIRED TRUE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(Boost_INCLUDE_DIR "/vcpkg/installed/x64-linux/include")
set(Boost_LIBRARY_DIR "/vcpkg/installed/x64-linux/lib")
include_directories("/workspaces/RobotV3/MainCore/include")
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
link_directories("/workspaces/RobotV3/bin")
link_libraries("/workspaces/RobotV3/bin/libMainCore.a")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)

include_directories(include)
add_library(lidar_log_reader src/lidar_log_reader.cpp)
add_library(kinematics src/kinematics.cpp)
ament_target_dependencies(kinematics sensor_msgs)

add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher rclcpp tf2 tf2_ros geometry_msgs nav_msgs)
target_include_directories(odom_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(odom_publisher rt kinematics)

install(TARGETS odom_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(lidar_publisher src/lidar_publisher.cpp)
ament_target_dependencies(lidar_publisher rclcpp sensor_msgs)
target_include_directories(lidar_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lidar_publisher lidar_log_reader)

install(TARGETS lidar_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(minimal_publisher src/minimal_publisher.cpp)
ament_target_dependencies(minimal_publisher rclcpp std_msgs nav_msgs tf2_ros geometry_msgs)
target_include_directories(minimal_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS minimal_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY descriptions
  DESTINATION share/${PROJECT_NAME}
)

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
