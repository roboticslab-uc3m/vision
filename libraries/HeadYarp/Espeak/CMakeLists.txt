# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_DEVICE(Espeak TYPE teo::Espeak INCLUDE Espeak.hpp)

IF (NOT SKIP_Espeak)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(Espeak REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_HEAD_INCLUDE_DIRS} ${ESPEAK_INCLUDE_DIRS})

yarp_add_idl(IDL_GEN_FILES speech.thrift)

YARP_ADD_PLUGIN(Espeak Espeak.hpp Espeak.cpp ${IDL_GEN_FILES})
TARGET_LINK_LIBRARIES(Espeak ${YARP_LIBRARIES} ${ESPEAK_LIBRARIES})
install(TARGETS Espeak DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_HEAD_INCLUDE_DIRS ${TEO_HEAD_INCLUDE_DIRS} ${ESPEAK_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
set(TEO_HEAD_LIBRARIES ${TEO_HEAD_LIBRARIES} HeadYarp Espeak ${ESPEAK_LIBRARIES} CACHE INTERNAL "appended libraries")

ENDIF (NOT SKIP_Espeak)

