cmake_minimum_required(VERSION 3.19..3.28)

project(exampleRemoteRGBDSensor LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.11 REQUIRED COMPONENTS os dev sig)
endif()

add_executable(exampleRemoteRGBDSensor exampleRemoteRGBDSensor.cpp)

target_link_libraries(exampleRemoteRGBDSensor YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig)
