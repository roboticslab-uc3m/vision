if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_switchDetection OR ENABLE_switchDetection))
    message(WARNING "OpenCV package not found, disabling colorRegionDetection program")
endif()

cmake_dependent_option(ENABLE_switchDetection "Enable/disable switchDetection program" ON
                       OpenCV_FOUND OFF)

if(ENABLE_switchDetection)

    include_directories(${OpenCV_INCLUDE_DIRS})

    add_executable(switchDetection main.cpp
                                   SwitchDetection.hpp
                                   SwitchDetection.cpp
                                   SegmentorThread.hpp
                                   SegmentorThread.cpp)

    target_link_libraries(switchDetection YARP::YARP_os
                                          YARP::YARP_init
                                          YARP::YARP_dev
                                          YARP::YARP_sig
                                          ${OpenCV_LIBS})

    install(TARGETS switchDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_switchDetection OFF CACHE BOOL "Enable/disable switchDetection program" FORCE)

endif()
