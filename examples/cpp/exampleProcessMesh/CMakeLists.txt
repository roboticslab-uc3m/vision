cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleProcessMesh LANGUAGES CXX)

find_package(YARP 3.3 REQUIRED os)
find_package(ROBOTICSLAB_VISION REQUIRED)

add_executable(exampleProcessMesh main.cpp)

target_link_libraries(exampleProcessMesh YARP::YARP_os
                                         ROBOTICSLAB::YarpCloudUtils)
