# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

IF (ENABLE_VisionYarp)

  option(ENABLE_launchVisionYarp "Choose if you want to compile launchVisionYarp" TRUE)

  find_package(YARP REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS})
  list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})  
  include(YarpPlugin)

#  yarp_begin_plugin_library(VisionYarp)
   add_subdirectory(Espeak)
#  yarp_end_plugin_library(VisionYarp)

#  install(TARGETS VisionYarp DESTINATION lib)

#  IF (ENABLE_launchVisionYarp)

#    yarp_add_plugin_library_executable(launchVisionYarp VisionYarp)
#    install(TARGETS launchVisionYarp DESTINATION bin)

#  ENDIF (ENABLE_launchVisionYarp)

ENDIF (ENABLE_VisionYarp)
