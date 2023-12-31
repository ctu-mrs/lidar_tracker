cmake_minimum_required(VERSION 3.1.2)
project(lidar_tracker)

# set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options("-DEIGEN_INITIALIZE_MATRICES_BY_NAN")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  message_generation
  message_runtime
  dynamic_reconfigure
  image_transport
  image_geometry
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  mrs_msgs
  mrs_lib
  ouster_ros
  vofod
)

generate_dynamic_reconfigure_options(
  cfg/covmat.cfg
)

add_message_files(DIRECTORY msg FILES
  Track.msg
  Tracks.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LidarTracker
  CATKIN_DEPENDS
  roscpp
  nodelet
  message_runtime
  dynamic_reconfigure
  mrs_lib
  image_transport
  mrs_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  ouster_ros
  vofod
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wpedantic)
add_definitions(-g)

add_library(LidarTracker src/lidar_tracker.cpp)
add_dependencies(LidarTracker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )
target_link_libraries(LidarTracker
  ${catkin_LIBRARIES}
  )
