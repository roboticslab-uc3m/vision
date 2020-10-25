if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_ColorRegionDetector OR ENABLE_ColorRegionDetector))
    message(WARNING "OpenCV package not found, disabling ColorRegionDetector library")
endif()

cmake_dependent_option(ENABLE_ColorRegionDetector "Enable/disable ColorRegionDetector" ON
                       "ENABLE_TravisLib;OpenCV_FOUND" OFF)

yarp_prepare_plugin(ColorRegionDetector
                    CATEGORY device
                    TYPE roboticslab::ColorRegionDetector
                    INCLUDE ColorRegionDetector.hpp
                    DEFAULT ON)

if(NOT SKIP_ColorRegionDetector)

    yarp_add_plugin(ColorRegionDetector ColorRegionDetector.hpp
                                        ColorRegionDetector.cpp)

    target_link_libraries(ColorRegionDetector PUBLIC YARP::YARP_os
                                                     VisionInterfaces
                                              PRIVATE YARP::YARP_cv
                                                      ${OpenCV_LIBS}
                                                      ROBOTICSLAB::ColorDebug
                                                      TravisLib)

    yarp_install(TARGETS ColorRegionDetector
                 EXPORT ROBOTICSLAB_VISION
                 LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
                 ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
                 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
                 PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

else()

    set(ENABLE_ColorRegionDetector OFF CACHE BOOL "Enable/disable ColorRegionDetector library" FORCE)

endif()
