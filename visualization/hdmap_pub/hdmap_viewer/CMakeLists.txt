cmake_minimum_required(VERSION 2.8.3)
project(hdmap_viewer)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        pcl_conversions
        pcl_msgs
        #  pcl_ros
        sensor_msgs
        std_msgs
        message_generation)
set(PCL_DIR "/home/lfg/pcl/PCLConfig.cmake")

find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
find_package(OpenCV)
include_directories(include ${OpenCV_INCLUDE_DIRS})
#include_directories(/usr/local/include )

#set(PCL_LIBRARY_DIRS /usr/local/lib)
link_directories(${PCL_LIBRARY_DIRS})
#link_directories( /usr/local/lib)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_message_files(
        FILES
        element.msg
        elements.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES pcl_deal
        #  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros sensor_msgs
        #  DEPENDS system_lib
)
catkin_package(
        DEPENDS system_lib
        CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pcl_deal.cpp
# )
add_definitions(${PCL_DEFINITIONS})
#set(${PCL_LIBRARIES}  /usr/local/lib/libpcl*.so)

add_executable(hdmap_viewer src/hdmap_viewer.cpp src/Utils.cc src/Utils.h)
target_link_libraries(hdmap_viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES}${Opencv_LIBS})

