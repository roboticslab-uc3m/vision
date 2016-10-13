# Copyright: Universidad Carlos III de Madrid (C) 2013
# Authors: Juan G. Victores

set(COLOR_DEBUG_PART_OF_PROJECT TRUE)
add_subdirectory(ColorDebug) # ColorDebug sets COLOR_DEBUG_INCLUDE_DIRS
set(TEO_HEAD_INCLUDE_DIRS ${TEO_HEAD_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

add_subdirectory(HeadYarp)  # HeadYarp a plugin library which contains YARP plugins

