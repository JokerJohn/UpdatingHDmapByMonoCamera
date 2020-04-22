cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME hdmap)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3)

#==========by hand
set(PROTO_FILES
        proto/HDMap.proto
        proto/LaneMarking.proto
        proto/SourceInfo.proto
        proto/TrafficLight.proto
        proto/Divider.proto
        )
set(SRC_FILES
#        src/test.cpp
#        src/Utils.cc
        src/main_zj.cpp
        )

set(HDR_FILES
        utils/build_depth.h
        utils/calulate.h
        utils/consts.h
        utils/pix2enu.h
        utils/pixel2enu.h
        utils/container.h
        utils/proto.h
        utils/read_detection.h
        utils/Utils.h
        utils/read_hdmap.h
        utils/vo.h
        utils/vo_extra.h)
add_definitions(${PCL_DEFINITIONS})

PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HDR ${PROTO_FILES})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${SRC_FILES} ${HDR_FILES} ${PROTO_SRC} ${PROTO_HDR} ${PROTO_FILES})
target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC ${PROTOBUF_INCLUDE_DIRS}
        #        ${Sophus_INCLUDE_DIRS}
        )
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARY_DIRS}
        ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${Sophus_LIBRARIES}#[[ /usr/local/lib/libSophus.so]]
        )

