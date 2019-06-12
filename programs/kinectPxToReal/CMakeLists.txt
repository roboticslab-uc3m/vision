option(ENABLE_kinectPxToReal "Enable/disable kinectPxToReal program" ON)

if(ENABLE_kinectPxToReal)

add_executable(kinectPxToReal main.cpp
                              KinectPxToReal.hpp
                              KinectPxToReal.cpp
                              CallbackPort.hpp
                              CallbackPort.cpp)

target_link_libraries(kinectPxToReal YARP::YARP_OS
                                     YARP::YARP_init
                                     YARP::YARP_dev)

install(TARGETS kinectPxToReal
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
