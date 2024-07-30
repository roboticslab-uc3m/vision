cmake_minimum_required(VERSION 3.16..3.28)

project(exampleArucoDetector LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.8 REQUIRED COMPONENTS os dev sig)
endif()

if(NOT TARGET ROBOTICSLAB::VisionInterfaces)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleArucoDetector exampleArucoDetector.cpp)

target_link_libraries(exampleArucoDetector YARP::YARP_os
                                           YARP::YARP_init
                                           YARP::YARP_dev
                                           YARP::YARP_sig
                                           ROBOTICSLAB::VisionInterfaces)
