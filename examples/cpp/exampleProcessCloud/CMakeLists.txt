cmake_minimum_required(VERSION 3.16..3.28)

project(exampleProcessCloud LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.8 REQUIRED COMPONENTS os)
endif()

if(NOT TARGET ROBOTICSLAB::YarpCloudUtils)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleProcessCloud main.cpp)

target_link_libraries(exampleProcessCloud YARP::YARP_os
                                          ROBOTICSLAB::YarpCloudUtils)
