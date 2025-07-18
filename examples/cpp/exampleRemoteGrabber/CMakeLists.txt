cmake_minimum_required(VERSION 3.19..3.28)

project(exampleRemoteGrabber LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.11 REQUIRED COMPONENTS os dev sig)
endif()

add_executable(exampleRemoteGrabber exampleRemoteGrabber.cpp)

target_link_libraries(exampleRemoteGrabber YARP::YARP_os
                                           YARP::YARP_init
                                           YARP::YARP_dev
                                           YARP::YARP_sig)
