cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleMeshFromLiveDepth LANGUAGES CXX)

find_package(YARP 3.3 REQUIRED os dev sig)
find_package(ROBOTICSLAB_VISION REQUIRED)

add_executable(exampleMeshFromLiveDepth main.cpp)

target_link_libraries(exampleMeshFromLiveDepth YARP::YARP_os
                                           YARP::YARP_init
                                           YARP::YARP_dev
                                           YARP::YARP_sig
                                           ROBOTICSLAB::YarpCloudUtils)
