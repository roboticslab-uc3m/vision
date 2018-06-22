if(NOT DEFINED ENABLE_colorRegionDetection OR ENABLE_colorRegionDetection)
    if(NOT OpenCV_FOUND)
        message(WARNING "OpenCV package not found, disabling colorRegionDetection program")
    endif()

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3)
        message(WARNING "Unsuitable YARP version (>=3), disabling colorRegionDetection program")
    endif()
endif()

cmake_dependent_option(ENABLE_colorRegionDetection "Enable/disable colorRegionDetection program" ON
                       "ENABLE_TravisLib;OpenCV_FOUND;YARP_VERSION_SHORT VERSION_LESS 3" OFF)

if(ENABLE_colorRegionDetection)

    include_directories(${OpenCV_INCLUDE_DIRS})

    add_executable(colorRegionDetection main.cpp
                                        ColorRegionDetection.hpp
                                        ColorRegionDetection.cpp
                                        SegmentorThread.hpp
                                        SegmentorThread.cpp)

    target_link_libraries(colorRegionDetection YARP::YARP_OS
                                               YARP::YARP_init
                                               YARP::YARP_dev
                                               YARP::YARP_sig
                                               ${OpenCV_LIBS}
                                               TravisLib)

    install(TARGETS colorRegionDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_colorRegionDetection OFF CACHE BOOL "Enable/disable colorRegionDetection program" FORCE)

endif()
