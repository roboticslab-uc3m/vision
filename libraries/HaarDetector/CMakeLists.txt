if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_switchDetection2D OR ENABLE_switchDetection2D))
    message(WARNING "OpenCV package not found, disabling switchDetection2D program")
endif()

#cmake_dependent_option(ENABLE_HaarDetector "Enable/disable HaarDetector" ON
#                       OpenCV_FOUND OFF)

#if(ENABLE_HaarDetector)

    include_directories(${OpenCV_INCLUDE_DIRS})

    add_library(HaarDetector HaarDetector.hpp
                             HaarDetector.cpp)

    target_link_libraries(HaarDetector YARP::YARP_OS
                                       YARP::YARP_init
                                       YARP::YARP_dev
                                       YARP::YARP_sig
                                       ${OpenCV_LIBS}
                                       ROBOTICSLAB::ColorDebug
                                       DetectorInterfaces)

     install(TARGETS HaarDetector
            DESTINATION ${CMAKE_INSTALL_BINDIR})

#else()

#    set(ENABLE_HaarDetector OFF CACHE BOOL "Enable/disable HaarDetector library" FORCE)

#endif()
