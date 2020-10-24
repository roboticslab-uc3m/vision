if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_ColorRegionDetector OR ENABLE_ColorRegionDetector))
    message(WARNING "OpenCV package not found, disabling ColorRegionDetector library")
endif()

cmake_dependent_option(ENABLE_ColorRegionDetector "Enable/disable ColorRegionDetector" ON
                       "ENABLE_TravisLib;OpenCV_FOUND" OFF)

if(ENABLE_ColorRegionDetector)

    add_library(ColorRegionDetector ColorRegionDetector.hpp
                                    ColorRegionDetector.cpp)

    set_target_properties(ColorRegionDetector PROPERTIES PUBLIC_HEADER ColorRegionDetector.hpp)

    target_include_directories(ColorRegionDetector PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                          $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    target_link_libraries(ColorRegionDetector PUBLIC YARP::YARP_os
                                                     DetectorInterfaces
                                              PRIVATE YARP::YARP_cv
                                                      ${OpenCV_LIBS}
                                                      ROBOTICSLAB::ColorDebug
                                                      TravisLib)

    install(TARGETS ColorRegionDetector
            EXPORT ROBOTICSLAB_VISION
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    set_property(GLOBAL APPEND PROPERTY _exported_targets ColorRegionDetector)

else()

    set(ENABLE_ColorRegionDetector OFF CACHE BOOL "Enable/disable ColorRegionDetector library" FORCE)

endif()
