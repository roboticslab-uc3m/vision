# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

option(ENABLE_YarpPlugins "Enable/disable YarpPlugins" OFF)

if(ENABLE_YarpPlugins)

# Find YARP (common dependency for all devices).
find_package(YARP REQUIRED)

# Include YARP CMake modules (needless since YARP 2.3.70).
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# Utilities for creating YARP devices.
include(YarpPlugin)

# Common YARP includes for subdirectories (needless since YARP 2.3.70).
include_directories(${YARP_INCLUDE_DIRS})

# Make interface headers discoverable by subdirectories.
#include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# YARP devices.
#add_subdirectory(...)

# Install interface headers.
#install(FILES IWhatever.h
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export variables to the global scope.
#set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_VISION_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

endif()
