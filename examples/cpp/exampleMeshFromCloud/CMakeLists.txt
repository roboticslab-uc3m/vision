cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleMeshFromCloud LANGUAGES CXX)

find_package(YARP 3.3 REQUIRED os)

if(NOT TARGET ROBOTICSLAB::YarpCloudUtils)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleMeshFromCloud main.cpp)

target_link_libraries(exampleMeshFromCloud YARP::YARP_os
                                           ROBOTICSLAB::YarpCloudUtils)
