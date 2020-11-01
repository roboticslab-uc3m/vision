option(ENABLE_switchDetection2D "Enable/disable switchDetection2D program" ON)

if(ENABLE_switchDetection2D)

    # switchDetection2D
    add_executable(switchDetection2D main.cpp
                                     SwitchDetection2D.hpp
                                     SwitchDetection2D.cpp
                                     DetectorThread.hpp
                                     DetectorThread.cpp)

    target_link_libraries(switchDetection2D YARP::YARP_os
                                            YARP::YARP_init
                                            YARP::YARP_dev
                                            YARP::YARP_sig
                                            ROBOTICSLAB::ColorDebug
                                            VisionInterfaces)

    install(TARGETS switchDetection2D
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_switchDetection2D OFF CACHE BOOL "Enable/disable switchDetection2D program" FORCE)

endif()
