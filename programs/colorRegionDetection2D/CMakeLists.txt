if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_colorRegionDetection2D OR ENABLE_colorRegionDetection2D))
    message(WARNING "OpenCV package not found, disabling colorRegionDetection2D program")
endif()

cmake_dependent_option(ENABLE_colorRegionDetection2D "Enable/disable colorRegionDetection2D program" ON
                       "ENABLE_TravisLib;OpenCV_FOUND" OFF)

if(ENABLE_colorRegionDetection2D)

    include_directories(${OpenCV_INCLUDE_DIRS})

    add_executable(colorRegionDetection2D main.cpp
                                          SegmentorThread.cpp
                                          SegmentorThread.hpp
                                          ColorRegionDetection2D.cpp
                                          ColorRegionDetection2D.hpp)

    target_link_libraries(colorRegionDetection2D YARP::YARP_OS
                                                 YARP::YARP_init
                                                 YARP::YARP_dev
                                                 YARP::YARP_sig
                                                 ${OpenCV_LIBS}
                                                 TravisLib)

    install(TARGETS colorRegionDetection2D
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_colorRegionDetection2D OFF CACHE BOOL "Enable/disable colorRegionDetection2D program" FORCE)

endif()
