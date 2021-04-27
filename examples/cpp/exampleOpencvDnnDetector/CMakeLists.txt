cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleOpencvDnnDetector LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os dev sig)
endif()

if(NOT TARGET ROBOTICSLAB::VisionInterfaces)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleOpencvDnnDetector exampleOpencvDnnDetector.cpp)

target_link_libraries(exampleOpencvDnnDetector YARP::YARP_os
                                               YARP::YARP_init
                                               YARP::YARP_dev
                                               YARP::YARP_sig
                                               ROBOTICSLAB::VisionInterfaces)
