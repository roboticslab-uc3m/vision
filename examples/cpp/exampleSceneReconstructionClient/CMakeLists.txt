cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleSceneReconstructionClient LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os sig)
endif()

if(NOT TARGET ROBOTICSLAB::YarpCloudUtils)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleSceneReconstructionClient main.cpp)

target_link_libraries(exampleSceneReconstructionClient YARP::YARP_os
                                                       YARP::YARP_init
                                                       YARP::YARP_sig
                                                       ROBOTICSLAB::YarpCloudUtils)
