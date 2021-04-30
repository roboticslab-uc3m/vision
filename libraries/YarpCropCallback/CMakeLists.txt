# default-enabled

add_library(YarpCropCallback STATIC YarpCropCallback.hpp
                                    YarpCropCallback.cpp)

target_link_libraries(YarpCropCallback PUBLIC YARP::YARP_os
                                              YARP::YARP_sig)

target_include_directories(YarpCropCallback PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# no install, no export

add_library(ROBOTICSLAB::YarpCropCallback ALIAS YarpCropCallback)
