option(ENABLE_rgbdDetection "Enable/disable rgbdDetection program" ON)

if(ENABLE_rgbdDetection)

    add_executable(rgbdDetection main.cpp
                                 RgbdDetection.hpp
                                 RgbdDetection.cpp)

    target_link_libraries(rgbdDetection YARP::YARP_os
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        ROBOTICSLAB::VisionInterfaces)

    install(TARGETS rgbdDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_rgbdDetection OFF CACHE BOOL "Enable/disable rgbdDetection program" FORCE)

endif()
