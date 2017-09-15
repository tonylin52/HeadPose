cmake_minimum_required(VERSION 2.8)

PROJECT(HeadPose)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(../3rdParty/dlib/cmake)

find_package (OpenCV REQUIRED)
find_package (Boost REQUIRED)
find_package (TBB REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS} /lib)
    
link_directories(${OpenCV_LIB_DIR})

ADD_EXECUTABLE(headpose main.cpp HeadPose.cpp HeadPose.h HeadPoseExtern.cpp HeadPoseExtern.h)
add_library(HeadPose SHARED HeadPose.cpp HeadPose.h HeadPoseExtern.cpp HeadPoseExtern.h)

TARGET_LINK_LIBRARIES(headpose  dlib ${OpenCV_LIBS} ${TBB_LIBRARIES} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(HeadPose  dlib ${OpenCV_LIBS} ${TBB_LIBRARIES} ${Boost_LIBRARIES})