cmake_minimum_required(VERSION 3.19..3.28)

project(exampleQrDetector LANGUAGES CXX)

if(NOT YARP_FOUND)
    set(_yarp_components os dev sig)
    find_package(YARP 3.12 QUIET COMPONENTS ${_yarp_components})

    if(NOT YARP_FOUND)
        find_package(YARP 4.0 REQUIRED COMPONENTS ${_yarp_components})
    endif()
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
