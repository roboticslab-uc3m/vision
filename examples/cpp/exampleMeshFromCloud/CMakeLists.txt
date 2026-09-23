cmake_minimum_required(VERSION 3.19..3.28)

project(exampleMeshFromCloud LANGUAGES CXX)

if(NOT YARP_FOUND)
    set(_yarp_components os)
    find_package(YARP 3.12 QUIET COMPONENTS ${_yarp_components})

    if(NOT YARP_FOUND)
        find_package(YARP 4.0 REQUIRED COMPONENTS ${_yarp_components})
    endif()
endif()

if(NOT TARGET ROBOTICSLAB::YarpCloudUtils)
    find_package(ROBOTICSLAB_VISION REQUIRED)
endif()

add_executable(exampleMeshFromCloud main.cpp)

target_link_libraries(exampleMeshFromCloud YARP::YARP_os
                                           ROBOTICSLAB::YarpCloudUtils)
