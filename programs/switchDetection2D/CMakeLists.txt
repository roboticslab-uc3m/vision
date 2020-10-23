if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_switchDetection2D OR ENABLE_switchDetection2D))
    message(WARNING "OpenCV package not found, disabling switchDetection2D program")
endif()

cmake_dependent_option(ENABLE_switchDetection2D "Enable/disable switchDetection2D program" ON
                                                "ENABLE_TravisLib;OpenCV_FOUND" OFF)

if(ENABLE_switchDetection2D)

    # switchDetection2D
    add_executable(switchDetection2D main.cpp
                                     SwitchDetection2D.hpp
                                     SwitchDetection2D.cpp
                                     DetectorThread.hpp
                                     DetectorThread.cpp)

    target_link_libraries(switchDetection2D YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_dev
                                            YARP::YARP_sig
                                            ROBOTICSLAB::ColorDebug
                                            ColorRegionDetector
                                            HaarDetector
                                            DetectorInterfaces)

    install(TARGETS switchDetection2D
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_switchDetection2D OFF CACHE BOOL "Enable/disable switchDetection2D program" FORCE)

endif()
