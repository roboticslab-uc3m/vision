cmake_minimum_required(VERSION 3.19..3.28)

project(exampleSceneReconstructionClient LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.11 REQUIRED COMPONENTS os sig)
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

target_compile_definitions(exampleSceneReconstructionClient PRIVATE SAMPLE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/pipeline.ini")
