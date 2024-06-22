cmake_minimum_required(VERSION 3.16)

project(exampleSceneReconstructionClient LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.8 REQUIRED COMPONENTS os sig)
endif()

if(NOT TARGET ROBOTICSLAB::YarpCloudUtils OR NOT TARGET ROBOTICSLAB::VisionIDL)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleSceneReconstructionClient main.cpp)

target_link_libraries(exampleSceneReconstructionClient YARP::YARP_os
                                                       YARP::YARP_init
                                                       YARP::YARP_sig
                                                       ROBOTICSLAB::YarpCloudUtils
                                                       ROBOTICSLAB::VisionIDL)
