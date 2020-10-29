# Created by Juan G Victores 2013 @ UC3M
# Thanks to Paul Fitzpatrick for all the YARP and iCub SWIG stuff for inspiration

cmake_minimum_required(VERSION 3.12)

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
set(CMAKE_CXX_STANDARD 11) # alternatively would get via YARP target property

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

set(CREATE_PYTHON FALSE CACHE BOOL "Create Python interface")

FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})
MESSAGE(STATUS "${SWIG_USE_FILE}")
SET (MIN_SWIG_VERSION "1.3.29")
IF ("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")
  MESSAGE( FATAL_ERROR "Please update your SWIG. Current version: ${SWIG_VERSION}. The minimum version needed: ${MIN_SWIG_VERSION}")
ENDIF ("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")

SET_SOURCE_FILES_PROPERTIES(roboticslab_vision.i PROPERTIES CPLUSPLUS ON)

if (CREATE_PYTHON)
  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}")
  SET(CMAKE_SWIG_FLAGS "-Wall;-module;roboticslab_vision;-threads")
  SET(CREATE_PYTHON_VERSION "" CACHE STRING "Control python version used, if you wish." )
  FIND_PACKAGE(PythonLibs ${CREATE_PYTHON_VERSION})
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

  LINK_LIBRARIES(${PYTHON_LIBRARY})
  
  ## in Windows it seems necessary to declare explictly the link_directory
  if (WIN32)
    get_filename_component(PYTHON_DIR ${PYTHON_LIBRARY} PATH)
    link_directories(${PYTHON_DIR})
  endif()
  
  #SWIG_ADD_MODULE(roboticslab_vision python roboticslab_vision.i)
  swig_add_library(roboticslab_vision
                   LANGUAGE python
                   SOURCES roboticslab_vision.i)
  #swig_link_libraries(roboticslab_vision ${PYTHON_LIBRARY} VisionInterfaces)
  #target_include_directories(roboticslab_vision SYSTEM PRIVATE ${PYTHON_INCLUDE_PATH})
  #swig_link_libraries(roboticslab_vision ${PYTHON_LIBRARY} YARP_os YARP_sig)

  # uncomment if we want a debug version of the library
  #ADD_DEFINITIONS(-DPy_DEBUG)

  # installation path is determined reliably on most platforms using distutils
  FIND_PACKAGE(PythonInterp ${CREATE_PYTHON_VERSION})

  EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} 
    -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix='${CMAKE_INSTALL_PREFIX}'))" 
    OUTPUT_VARIABLE PYTHON_INSTDIR OUTPUT_STRIP_TRAILING_WHITESPACE )
  INSTALL(FILES ${CMAKE_BINARY_DIR}/roboticslab_vision.py ${CMAKE_BINARY_DIR}/lib/_roboticslab_vision.so 
    DESTINATION ${PYTHON_INSTDIR} )
  #j#INSTALL(FILES ${CMAKE_BINARY_DIR}/roboticslab_vision.py
  #j#  DESTINATION ${PYTHON_INSTDIR} )

  IF (WIN32)
    # Check if we have a target called _roboticslab_vision as SWIG_ADD_MODULE
    # will currently give. If not, we're probably in the distant
    # future, and we'd best not meddle.
    GET_TARGET_PROPERTY(roboticslab_vision_lib_location _roboticslab_vision LOCATION)
    IF (roboticslab_vision_lib_location)
      SET_TARGET_PROPERTIES(_roboticslab_vision PROPERTIES SUFFIX ".pyd")
    ENDIF ()    
  ENDIF (WIN32)
endif (CREATE_PYTHON)
