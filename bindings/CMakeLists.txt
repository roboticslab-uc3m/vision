# Created by Juan G Victores 2013 @ UC3M
# Thanks to Paul Fitzpatrick for all the YARP and iCub SWIG stuff for inspiration

# UseSWIG generates now standard target names.
if(${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.13)
  cmake_policy(SET CMP0078 OLD)
endif()

include(CMakeDependentOption)
include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

# disable all warnings for current folder and subfolders
set(CMAKE_C_FLAGS -w)
set(CMAKE_CXX_FLAGS -w)

get_target_property(_vision_interface_include_dirs VisionInterfaces INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${_vision_interface_include_dirs})

set(_components conf os sig dev)
# This is necessary for SWIG to parse yarp.i file
# TODO This might no longer be necessary with some updated UseSWIG.cmake version
foreach(_component IN LISTS _components)
  get_property(_include_dirs TARGET YARP::YARP_${_component} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  foreach(_dir IN LISTS _include_dirs)
    if("${_dir}" MATCHES "\\\$<BUILD_INTERFACE:(.+)>$")
      include_directories("${CMAKE_MATCH_1}")
    elseif("${_dir}" MATCHES "\\\$<INSTALL_INTERFACE:(.+)>$")
      # Nothing to do
    else()
      include_directories(${_dir})
    endif()
  endforeach()
endforeach()

option(CREATE_BINDINGS "Compile optional language bindings" TRUE)

if(CREATE_BINDINGS)

  set(SWIG_EXTRA_FLAGS "" CACHE STRING "Extra flags passed to swig commands (e.g. -Wall)")
  mark_as_advanced(SWIG_EXTRA_FLAGS)
  unset(SWIG_COMMON_FLAGS)
  list(APPEND SWIG_COMMON_FLAGS ${SWIG_EXTRA_FLAGS})

  set(CREATE_BINDINGS_PYTHON FALSE CACHE BOOL "Create Python interface")

  find_package(SWIG REQUIRED)
  include(${SWIG_USE_FILE})

  set(SWIG_BINDINGS_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/roboticslab_vision.i")

  set_source_files_properties(roboticslab_vision.i PROPERTIES CPLUSPLUS ON)

  if(CREATE_BINDINGS_PYTHON)
    add_subdirectory(python)
  endif()
endif()
