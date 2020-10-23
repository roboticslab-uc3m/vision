if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_HaarDetector OR ENABLE_HaarDetector))
    message(WARNING "OpenCV package not found, disabling HaarDetector library")
endif()

cmake_dependent_option(ENABLE_HaarDetector "Enable/disable HaarDetector" ON
                       OpenCV_FOUND OFF)

if(ENABLE_HaarDetector)

    add_library(HaarDetector HaarDetector.hpp
                             HaarDetector.cpp)

    set_target_properties(HaarDetector PROPERTIES PUBLIC_HEADER HaarDetector.hpp)

    target_include_directories(HaarDetector PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    target_link_libraries(HaarDetector PUBLIC YARP::YARP_os
                                              DetectorInterfaces
                                       PRIVATE ${OpenCV_LIBS}
                                               ROBOTICSLAB::ColorDebug)

    install(TARGETS HaarDetector
            EXPORT ROBOTICSLAB_VISION
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    set_property(GLOBAL APPEND PROPERTY _exported_targets HaarDetector)

else()

    set(ENABLE_HaarDetector OFF CACHE BOOL "Enable/disable HaarDetector library" FORCE)

endif()
