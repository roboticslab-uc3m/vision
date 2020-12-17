cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleColorRegionDetector LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS os dev sig)

if(NOT TARGET ROBOTICSLAB::VisionInterfaces)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleColorRegionDetector exampleColorRegionDetector.cpp)

target_link_libraries(exampleColorRegionDetector YARP::YARP_os
                                                 YARP::YARP_init
                                                 YARP::YARP_dev
                                                 YARP::YARP_sig
                                                 ROBOTICSLAB::VisionInterfaces)
