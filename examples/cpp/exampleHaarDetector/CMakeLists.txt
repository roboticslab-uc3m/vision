cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleHaarDetector LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS os dev sig)

if(NOT TARGET ROBOTICSLAB::VisionInterfaces)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleHaarDetector exampleHaarDetector.cpp)

target_link_libraries(exampleHaarDetector YARP::YARP_os
                                          YARP::YARP_init
                                          YARP::YARP_dev
                                          YARP::YARP_sig
                                          ROBOTICSLAB::VisionInterfaces)
