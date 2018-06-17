find_package(OpenCV QUIET)

if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_haarDetection2D OR ENABLE_haarDetection2D))
    message(WARNING "OpenCV package not found, disabling haarDetection2D program")
endif()

cmake_dependent_option(ENABLE_haarDetection2D "Enable/disable haarDetection2D program" ON
                       OpenCV_FOUND OFF)

if(ENABLE_haarDetection2D)

    include_directories(${OpenCV_INCLUDE_DIRS})

    if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
        include_directories(${YARP_INCLUDE_DIRS})
    endif()

    add_executable(haarDetection2D main.cpp
                                   HaarDetection2D.hpp
                                   HaarDetection2D.cpp
                                   SegmentorThread.hpp
                                   SegmentorThread.cpp)

    target_link_libraries(haarDetection2D YARP::YARP_OS
                                          YARP::YARP_init
                                          YARP::YARP_dev
                                          YARP::YARP_sig
                                          ${OpenCV_LIBS}
                                          ROBOTICSLAB::ColorDebug)

    install(TARGETS haarDetection2D
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_haarDetection2D OFF CACHE BOOL "Enable/disable haarDetection2D program" FORCE)

endif()
