# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

prepare_device(headbot TYPE teo::HeadBot INCLUDE HeadBot.hpp WRAPPER controlboard)

IF (NOT SKIP_headbot)

  find_package(YARP REQUIRED)

  # Include any directories needed for YARP
  include_directories(${YARP_INCLUDE_DIRS} ${HEAD_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS} /usr/realtime/include)

  yarp_add_plugin(headbot HeadBot.cpp )
  TARGET_LINK_LIBRARIES(headbot ${YARP_LIBRARIES})
  #install(TARGETS headbot DESTINATION lib)

ENDIF (NOT SKIP_headbot)

