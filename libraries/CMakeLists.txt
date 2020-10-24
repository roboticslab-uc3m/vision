# Copyright: Universidad Carlos III de Madrid (C) 2013
# Authors: Juan G. Victores

# Device interface target.
add_library(DetectorInterfaces INTERFACE)

get_target_property(_yarp_sig_iface_headers YARP::YARP_sig INTERFACE_INCLUDE_DIRECTORIES)

# Configure usage requirements.
target_include_directories(DetectorInterfaces INTERFACE ${_yarp_sig_iface_headers}
                                                        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Install interface headers.
install(FILES Detector.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Register export set.
install(TARGETS DetectorInterfaces
        EXPORT ROBOTICSLAB_VISION)

set_property(GLOBAL APPEND PROPERTY _exported_targets DetectorInterfaces)

add_subdirectory(TravisLib)
add_subdirectory(ColorRegionDetector)
add_subdirectory(HaarDetector)
add_subdirectory(YarpPlugins)
