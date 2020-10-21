# Copyright: Universidad Carlos III de Madrid (C) 2013
# Authors: Juan G. Victores

# Device interface target.
add_library(DetectorInterfaces INTERFACE)

# Configure usage requirements.
target_include_directories(DetectorInterfaces INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_subdirectory(ColorRegionDetector)
add_subdirectory(HaarDetector)
add_subdirectory(TravisLib)
add_subdirectory(YarpPlugins)
