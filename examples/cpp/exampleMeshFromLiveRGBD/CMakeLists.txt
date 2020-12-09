cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleMeshFromLiveRGBD LANGUAGES CXX)

find_package(YARP 3.3 REQUIRED os dev sig)
find_package(ROBOTICSLAB_VISION REQUIRED)

add_executable(exampleMeshFromLiveRGBD main.cpp)

target_link_libraries(exampleMeshFromLiveRGBD YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig
                                              ROBOTICSLAB::YarpCloudUtils)

target_compile_definitions(exampleMeshFromLiveRGBD PRIVATE SAMPLE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/pipeline.ini")
