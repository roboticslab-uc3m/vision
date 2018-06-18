find_package(OpenCV QUIET)

if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_haarDetection OR ENABLE_haarDetection))
    message(WARNING "OpenCV package not found, disabling colorRegionDetection program")
endif()

cmake_dependent_option(ENABLE_haarDetection "Enable/disable haarDetection program" ON
                       OpenCV_FOUND OFF)

if(ENABLE_haarDetection)

    include_directories(${OpenCV_INCLUDE_DIRS})

    add_executable(haarDetection main.cpp
                                 HaarDetection.hpp
                                 HaarDetection.cpp
                                 SegmentorThread.hpp
                                 SegmentorThread.cpp)

    target_link_libraries(haarDetection YARP::YARP_OS
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        ${OpenCV_LIBS})

    install(TARGETS haarDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_haarDetection OFF CACHE BOOL "Enable/disable haarDetection program" FORCE)

endif()
