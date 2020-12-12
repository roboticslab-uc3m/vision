cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleProcessCloud LANGUAGES CXX)

find_package(YARP 3.3 REQUIRED os)
find_package(ROBOTICSLAB_VISION REQUIRED)

add_executable(exampleProcessCloud main.cpp)

target_link_libraries(exampleProcessCloud YARP::YARP_os
                                          ROBOTICSLAB::YarpCloudUtils)
