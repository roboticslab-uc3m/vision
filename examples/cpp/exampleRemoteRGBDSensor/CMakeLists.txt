cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleRemoteRGBDSensor LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS os dev sig)

add_executable(exampleRemoteRGBDSensor exampleRemoteRGBDSensor.cpp)

target_link_libraries(exampleRemoteRGBDSensor YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig)
