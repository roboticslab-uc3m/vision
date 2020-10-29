# Created by Juan G Victores 2013 @ UC3M
# Thanks to Paul Fitzpatrick for all the YARP and iCub SWIG stuff for inspiration

# this is set in the parent directory, but for some reason it does not get inherited
set_source_files_properties(${SWIG_BINDINGS_SOURCE_FILE} PROPERTIES CPLUSPLUS ON)

set(CMAKE_SWIG_FLAGS "-module;roboticslab_vision;-threads;${SWIG_COMMON_FLAGS}")
set(VISION_USE_PYTHON_VERSION "" CACHE STRING "Specify python version to use" )
mark_as_advanced(VISION_USE_PYTHON_VERSION)
find_package(PythonInterp ${VISION_USE_PYTHON_VERSION} REQUIRED)
set(VISION_USE_PYTHON_VERSION_ARGS)
if(NOT VISION_USE_PYTHON_VERSION)
  set (VISION_USE_PYTHON_VERSION ${PYTHON_VERSION_STRING})
endif()
find_package(PythonLibs ${VISION_USE_PYTHON_VERSION} EXACT)


## in Windows it seems necessary to declare explictly the link_directory
if(WIN32)
  get_filename_component(PYTHON_DIR ${PYTHON_LIBRARY} DIRECTORY)
  link_directories(${PYTHON_DIR})
endif()

set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")

swig_add_library(roboticslab_vision
                 LANGUAGE python
                 SOURCES ${SWIG_BINDINGS_SOURCE_FILE})
swig_link_libraries(roboticslab_vision ${PYTHON_LIBRARY}
                                       YARP::YARP_os
                                       YARP::YARP_sig
                                       VisionInterfaces)
#target_include_directories(roboticslab_vision SYSTEM PRIVATE ${PYTHON_INCLUDE_PATH})

set_target_properties(${SWIG_MODULE_roboticslab_vision_python_REAL_NAME} PROPERTIES OUTPUT_NAME "_roboticslab_vision"
                                                                      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python")

# installation path is determined reliably on most platforms using distutils
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
                OUTPUT_VARIABLE PYTHON_INSTDIR
                OUTPUT_STRIP_TRAILING_WHITESPACE )

install(FILES ${CMAKE_BINARY_DIR}/lib/python/roboticslab_vision.py
        DESTINATION ${PYTHON_INSTDIR})

set(_CMAKE_INSTALL_PYTHONDIR "${PYTHON_INSTDIR}")
set(CMAKE_INSTALL_PYTHONDIR ${_CMAKE_INSTALL_PYTHONDIR} CACHE PATH "python bindings (${_CMAKE_INSTALL_PYTHONDIR})")
mark_as_advanced(CMAKE_INSTALL_PYTHONDIR)
if(NOT IS_ABSOLUTE ${CMAKE_INSTALL_PYTHONDIR})
  set(CMAKE_INSTALL_FULL_PYTHONDIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_PYTHONDIR}")
else()
  set(CMAKE_INSTALL_FULL_PYTHONDIR "${CMAKE_INSTALL_PYTHONDIR}")
endif()

# Update RPATH
if(NOT CMAKE_SKIP_RPATH AND NOT CMAKE_SKIP_INSTALL_RPATH)
  file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_FULL_PYTHONDIR}" "${CMAKE_INSTALL_FULL_LIBDIR}")
  get_target_property(_current_rpath ${SWIG_MODULE_roboticslab_vision_REAL_NAME} INSTALL_RPATH)
  if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND _current_rpath "@loader_path/${_rel_path}")
  else()
    list(APPEND _current_rpath "\$ORIGIN/${_rel_path}")
  endif()
  set_target_properties(${SWIG_MODULE_yarp_python_REAL_NAME} PROPERTIES INSTALL_RPATH "${_current_rpath}")
endif()

install(TARGETS ${SWIG_MODULE_roboticslab_vision_REAL_NAME}
        DESTINATION ${PYTHON_INSTDIR})
