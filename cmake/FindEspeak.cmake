# - Try to find ESPEAK
# Once done this will define
#  ESPEAK_FOUND - System has ESPEAK
#  ESPEAK_INCLUDE_DIRS - The ESPEAK include directories
#  ESPEAK_LIBRARIES - The libraries needed to use ESPEAK
#  ESPEAK_DEFINITIONS - Compiler switches required for using ESPEAK

#set(ESPEAK_DEFINITIONS ${PC_LIBXML_CFLAGS_OTHER})

find_path(ESPEAK_INCLUDE_DIRS speak_lib.h
  $ENV{ESPEAK_DIR}
  $ENV{FFMPEG_DIR}/espeak
  /usr/local/include/espeak
  /usr/include/espeak
)

find_library(ESPEAK_LIBRARIES NAMES espeak libespeak )

mark_as_advanced(ESPEAK_INCLUDE_DIRS ESPEAK_LIBRARY )

set(ESPEAK_INCLUDE_DIRS ${ESPEAK_INCLUDE_DIR} )
set(ESPEAK_LIBRARIES ${ESPEAK_LIBRARY} )

