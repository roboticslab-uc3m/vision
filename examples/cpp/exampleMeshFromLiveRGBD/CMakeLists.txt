cmake_minimum_required(VERSION 3.19..3.28)

project(exampleMeshFromLiveRGBD LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.11 REQUIRED COMPONENTS os dev sig)
endif()

if(NOT TARGET ROBOTICSLAB::YarpCloudUtils)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleMeshFromLiveRGBD main.cpp)

target_link_libraries(exampleMeshFromLiveRGBD YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig
                                              ROBOTICSLAB::YarpCloudUtils)

target_compile_definitions(exampleMeshFromLiveRGBD PRIVATE SAMPLE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/pipeline.ini")
