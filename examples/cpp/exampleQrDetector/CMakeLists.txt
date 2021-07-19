cmake_minimum_required(VERSION 3.12)

project(exampleQrDetector LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.4 REQUIRED COMPONENTS os dev sig)
endif()

if(NOT TARGET ROBOTICSLAB::VisionInterfaces)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleQrDetector exampleQrDetector.cpp)

target_link_libraries(exampleQrDetector YARP::YARP_os
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        ROBOTICSLAB::VisionInterfaces)
