# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

IF (ENABLE_HeadYarp)

  option(ENABLE_testHeadYarp "Choose if you want to compile testHeadYarp" FALSE)

  find_package(YARP REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS})
  list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})  
  include(YarpPlugin)

  yarp_begin_plugin_library(HeadYarp)
#    add_subdirectory(headbot)
  yarp_end_plugin_library(HeadYarp)

  install(TARGETS HeadYarp DESTINATION lib)

  IF (ENABLE_testHeadYarp)

    yarp_add_plugin_library_executable(testHeadYarp HeadYarp)
    install(TARGETS testHeadYarp DESTINATION bin)

  ENDIF (ENABLE_testHeadYarp)

ENDIF (ENABLE_HeadYarp)

