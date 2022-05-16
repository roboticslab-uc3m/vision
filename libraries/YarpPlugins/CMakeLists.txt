# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# Create device interface target.
add_library(VisionInterfaces INTERFACE)

# Configure usage requirements.
target_link_libraries(VisionInterfaces INTERFACE YARP::YARP_os YARP::YARP_sig)

target_include_directories(VisionInterfaces INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Install interface headers.
install(FILES IDetector.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Register export set.
install(TARGETS VisionInterfaces
        EXPORT ROBOTICSLAB_VISION)

# Export variables to the global scope.
set_property(GLOBAL APPEND PROPERTY _exported_targets VisionInterfaces)
set_property(GLOBAL APPEND PROPERTY _exported_dependencies YARP_sig)

# Create alias target.
add_library(ROBOTICSLAB::VisionInterfaces ALIAS VisionInterfaces)

# YARP devices.
add_subdirectory(ColorRegionDetector)
add_subdirectory(DnnDetector)
add_subdirectory(HaarDetector)
add_subdirectory(QrDetector)
add_subdirectory(HogFaceDetector)
