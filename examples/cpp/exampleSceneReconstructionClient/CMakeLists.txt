cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleSceneReconstructionClient LANGUAGES CXX)

find_package(YARP 3.3 REQUIRED os sig)
find_package(ROBOTICSLAB_VISION REQUIRED)

add_executable(exampleSceneReconstructionClient main.cpp)

target_link_libraries(exampleSceneReconstructionClient YARP::YARP_os
                                                       YARP::YARP_init
                                                       YARP::YARP_sig
                                                       ROBOTICSLAB::YarpCloudUtils)
