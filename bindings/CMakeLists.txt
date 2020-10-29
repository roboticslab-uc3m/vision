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

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
message(STATUS "${SWIG_USE_FILE}")

set_source_files_properties(roboticslab_vision.i PROPERTIES CPLUSPLUS ON)

if (CREATE_PYTHON)

  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}")
  set(CMAKE_SWIG_FLAGS "-Wall;-module;roboticslab_vision;-threads")
  set(CREATE_PYTHON_VERSION "" CACHE STRING "Control python version used, if you wish." )

  find_package(PythonLibs ${CREATE_PYTHON_VERSION})
  include_directories(${PYTHON_INCLUDE_PATH})

  link_libraries(${PYTHON_LIBRARY})
  
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
  swig_link_libraries(roboticslab_vision ${PYTHON_LIBRARY}
                                         YARP::YARP_os
                                         YARP::YARP_sig)

  # uncomment if we want a debug version of the library
  #ADD_DEFINITIONS(-DPy_DEBUG)

  # installation path is determined reliably on most platforms using distutils
  find_package(PythonInterp ${CREATE_PYTHON_VERSION})

  execute_process(COMMAND ${PYTHON_EXECUTABLE} 
    -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix='${CMAKE_INSTALL_PREFIX}'))" 
    OUTPUT_VARIABLE PYTHON_INSTDIR OUTPUT_STRIP_TRAILING_WHITESPACE )
  install(FILES ${CMAKE_BINARY_DIR}/roboticslab_vision.py ${CMAKE_BINARY_DIR}/lib/_roboticslab_vision.so 
    DESTINATION ${PYTHON_INSTDIR} )
  #j#INSTALL(FILES ${CMAKE_BINARY_DIR}/roboticslab_vision.py
  #j#  DESTINATION ${PYTHON_INSTDIR} )

  if (WIN32)
    # Check if we have a target called _roboticslab_vision as SWIG_ADD_MODULE
    # will currently give. If not, we're probably in the distant
    # future, and we'd best not meddle.
    get_target_property(roboticslab_vision_lib_location _roboticslab_vision LOCATION)
    if (roboticslab_vision_lib_location)
      set_target_properties(_roboticslab_vision PROPERTIES SUFFIX ".pyd")
    endif ()    
  endif (WIN32)
endif (CREATE_PYTHON)
