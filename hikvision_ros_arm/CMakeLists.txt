cmake_minimum_required(VERSION 3.5.0)
project(hikvision_ros)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # cv_bridge
  image_transport
  camera_info_manager
)

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDES})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDES})

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(hik_cam_node src/hik_cam.cpp)
# add_library(hik_cam_node src/hik_cam.cpp)
target_link_libraries(hik_cam_node
    ${catkin_LIBRARIES}
    # ${OPENCV_LIBS}
    hcnetsdk PlayCtrl
)

# add_executable(hik_cam_node node/hik_cam_node.cpp)
# target_link_libraries(hik_cam_node hik_cam)
