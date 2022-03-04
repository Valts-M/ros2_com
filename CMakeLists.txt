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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH 
  "/vcpkg/installed/x64-linux" 
  "/opt/ros/foxy/opt/yaml_cpp_vendor")

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
include_directories("/code/RobotV3/MainCore/include")
link_libraries("/code/RobotV3/bin/libMainCore.a" rt uuid)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
"srv/SaveMap.srv"
"srv/PauseOdom.srv"
"srv/ResetOdom.srv"
"srv/SendInitialPose.srv"
"srv/SaveInitialPose.srv"
"srv/PausePoseSend.srv"
"srv/CreateMapImg.srv"
DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_library(kinematics STATIC src/kinematics.cpp)
add_library(shmem_util STATIC src/shmem_util.cpp)
ament_target_dependencies(kinematics sensor_msgs)

set(libraries 
    kinematics
    shmem_util)

add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher rclcpp tf2 tf2_ros geometry_msgs nav_msgs)
target_include_directories(odom_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(odom_publisher shmem_util kinematics)
rosidl_target_interfaces(odom_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS odom_publisher
DESTINATION lib/${PROJECT_NAME})

add_executable(point2block src/point2block.cpp)
ament_target_dependencies(point2block rclcpp tf2 tf2_ros PCL sensor_msgs OpenCV)
target_include_directories(point2block PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(point2block shmem_util ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS point2block
DESTINATION lib/${PROJECT_NAME})

add_executable(pc_filter src/pc_filter.cpp)
ament_target_dependencies(pc_filter rclcpp PCL sensor_msgs)
target_include_directories(pc_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(pc_filter ${PCL_LIBRARIES})

install(TARGETS pc_filter
DESTINATION lib/${PROJECT_NAME})

add_executable(tf_filter src/tf_filter.cpp)
ament_target_dependencies(tf_filter rclcpp tf2 tf2_ros geometry_msgs)
target_include_directories(tf_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS tf_filter
  DESTINATION lib/${PROJECT_NAME})

# add_executable(slam_controller src/slam_controller.cpp)
# ament_target_dependencies(slam_controller rclcpp)
# target_include_directories(slam_controller PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(slam_controller shmem_util)
# rosidl_target_interfaces(slam_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install(TARGETS slam_controller
#   DESTINATION lib/${PROJECT_NAME})

add_executable(pose_listener src/pose_listener.cpp)
ament_target_dependencies(pose_listener rclcpp tf2 tf2_ros geometry_msgs)
target_include_directories(pose_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pose_listener shmem_util)
rosidl_target_interfaces(pose_listener ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS pose_listener
  DESTINATION lib/${PROJECT_NAME})

add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher rclcpp nav_msgs tf2 tf2_ros)
target_include_directories(path_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS path_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(clock_server src/clock.cpp)
ament_target_dependencies(clock_server rclcpp rosgraph_msgs)
target_include_directories(clock_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS clock_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(map_saver src/map_saver.cpp)
ament_target_dependencies(map_saver rclcpp nav_msgs OpenCV yaml-cpp) 
target_include_directories(map_saver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(map_saver shmem_util ${YAML_CPP_LIBRARIES})
rosidl_target_interfaces(map_saver ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS map_saver
  DESTINATION lib/${PROJECT_NAME})

add_executable(map_loader src/map_loader.cpp)
ament_target_dependencies(map_loader rclcpp nav_msgs OpenCV yaml-cpp) 
target_include_directories(map_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(map_loader shmem_util ${YAML_CPP_LIBRARIES})
rosidl_target_interfaces(map_loader ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS map_loader
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${libraries}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
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
