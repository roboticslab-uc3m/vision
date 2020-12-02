option(ENABLE_switchDetection "Enable/disable switchDetection program" ON)

if(ENABLE_switchDetection)

    add_executable(switchDetection main.cpp
                                   SwitchDetection.hpp
                                   SwitchDetection.cpp
                                   SegmentorThread.hpp
                                   SegmentorThread.cpp)

    target_link_libraries(switchDetection YARP::YARP_os
                                          YARP::YARP_init
                                          YARP::YARP_dev
                                          YARP::YARP_sig
                                          VisionInterfaces)

    install(TARGETS switchDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_switchDetection OFF CACHE BOOL "Enable/disable switchDetection program" FORCE)

endif()
