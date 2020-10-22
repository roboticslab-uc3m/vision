if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_switchDetection2D OR ENABLE_switchDetection2D))
    message(WARNING "OpenCV package not found, disabling switchDetection2D program")
endif()

#cmake_dependent_option(ENABLE_ColorRegionDetector "Enable/disable ColorRegionDetector" ON
#                       OpenCV_FOUND OFF)

#if(ENABLE_ColorRegionDetector)

    include_directories(${OpenCV_INCLUDE_DIRS})

    add_library(ColorRegionDetector ColorRegionDetector.hpp
                                    ColorRegionDetector.cpp)

    target_link_libraries(ColorRegionDetector YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig
                                              ${OpenCV_LIBS}
                                              ROBOTICSLAB::ColorDebug
                                              TravisLib
                                              DetectorInterfaces) # ROBOTICSLAB::TravisLib ? ...only finds installed header!!

    install(TARGETS ColorRegionDetector
            DESTINATION ${CMAKE_INSTALL_BINDIR})

#else()

#    set(ENABLE_ColorRegionDetector OFF CACHE BOOL "Enable/disable ColorRegionDetector library" FORCE)

#endif()
