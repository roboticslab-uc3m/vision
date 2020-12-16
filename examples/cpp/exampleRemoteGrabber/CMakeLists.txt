cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleRemoteGrabber LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS os dev sig)

add_executable(exampleRemoteGrabber exampleRemoteGrabber.cpp)

target_link_libraries(exampleRemoteGrabber YARP::YARP_os
                                           YARP::YARP_init
                                           YARP::YARP_dev
                                           YARP::YARP_sig)

include(GNUInstallDirs)

install(TARGETS exampleRemoteGrabber
        DESTINATION ${CMAKE_INSTALL_BINDIR})
