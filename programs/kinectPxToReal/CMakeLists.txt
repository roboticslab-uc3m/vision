option(ENABLE_kinectPxToReal "Enable/disable kinectPxToReal program" ON)

if(ENABLE_kinectPxToReal)

if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
    include_directories(${YARP_INCLUDE_DIRS})
endif()

add_executable(kinectPxToReal main.cpp
                              KinectPxToReal.hpp
                              KinectPxToReal.cpp
                              CallbackPort.hpp
                              CallbackPort.cpp)

target_link_libraries(kinectPxToReal YARP::YARP_OS
                                     YARP::YARP_init
                                     YARP::YARP_sig)

install(TARGETS kinectPxToReal
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
