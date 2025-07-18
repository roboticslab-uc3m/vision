cmake_minimum_required(VERSION 3.19..3.28)

project(exampleColorRegionDetector LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.11 REQUIRED COMPONENTS os dev sig)
endif()

if(NOT TARGET ROBOTICSLAB::VisionInterfaces)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleColorRegionDetector exampleColorRegionDetector.cpp)

target_link_libraries(exampleColorRegionDetector YARP::YARP_os
                                                 YARP::YARP_init
                                                 YARP::YARP_dev
                                                 YARP::YARP_sig
                                                 ROBOTICSLAB::VisionInterfaces)
