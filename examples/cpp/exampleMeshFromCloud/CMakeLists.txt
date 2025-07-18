cmake_minimum_required(VERSION 3.19..3.28)

project(exampleMeshFromCloud LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.11 REQUIRED COMPONENTS os)
endif()

if(NOT TARGET ROBOTICSLAB::YarpCloudUtils)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleMeshFromCloud main.cpp)

target_link_libraries(exampleMeshFromCloud YARP::YARP_os
                                           ROBOTICSLAB::YarpCloudUtils)
