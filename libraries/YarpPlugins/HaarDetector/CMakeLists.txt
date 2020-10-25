if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_HaarDetector OR ENABLE_HaarDetector))
    message(WARNING "OpenCV package not found, disabling HaarDetector library")
endif()

cmake_dependent_option(ENABLE_HaarDetector "Enable/disable HaarDetector" ON
                       OpenCV_FOUND OFF)

yarp_prepare_plugin(HaarDetector
                    CATEGORY device
                    TYPE roboticslab::HaarDetector
                    INCLUDE HaarDetector.hpp
                    DEFAULT ON)

if(NOT SKIP_HaarDetector)

    yarp_add_plugin(HaarDetector HaarDetector.hpp
                                 HaarDetector.cpp)

    target_link_libraries(HaarDetector PUBLIC YARP::YARP_os
                                              VisionInterfaces
                                       PRIVATE ${OpenCV_LIBS}
                                               ROBOTICSLAB::ColorDebug)

    install(TARGETS HaarDetector
            EXPORT ROBOTICSLAB_VISION
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

else()

    set(ENABLE_HaarDetector OFF CACHE BOOL "Enable/disable HaarDetector library" FORCE)

endif()
