cmake_minimum_required(VERSION 3.14)
project(deecamp)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_STANDARD 14)

set(PROTO_FILES
        proto/HDMap.proto
        proto/LaneMarking.proto
        proto/SourceInfo.proto
        proto/TrafficLight.proto
        proto/Divider.proto
        )

set(HDR_FILES utils/read_hdmap.h utils/calulate.h utils/proto.h)

set(PROTO_SRC
        proto/Divider.pb.cc
        proto/HDMap.pb.cc
        proto/LaneMarking.pb.cc
        proto/SourceInfo.pb.cc
        proto/TrafficLight.pb.cc
        proto/Divider.pb.cc
        )

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(/usr/local/protobuf/include)
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(deecamp main.cpp utils/Utils.cc utils/Utils.h ${HDR_FILES} ${PROTO_FILES} ${PROTO_SRC})
target_link_libraries(deecamp ${OpenCV_LIBS} ${EIGEN3_LIBS} /usr/local/protobuf/lib/libprotobuf.so)