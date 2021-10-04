# Created by Juan G Victores 2013 @ UC3M
# Thanks to Paul Fitzpatrick for all the YARP and iCub SWIG stuff for inspiration

# this is set in the parent directory, but for some reason it does not get inherited
set_source_files_properties(${SWIG_BINDINGS_SOURCE_FILE} PROPERTIES CPLUSPLUS ON)

set(CMAKE_SWIG_FLAGS "-module;roboticslab_vision;-threads;${SWIG_COMMON_FLAGS}")

# https://github.com/pybind/pybind11/pull/2689
if(${CMAKE_VERSION} VERSION_LESS "3.18.0")
  set(RL_Python_Development_Component "Development")
else()
  set(RL_Python_Development_Component "Development.Module")
endif()

find_package(Python3 COMPONENTS Interpreter ${RL_Python_Development_Component} REQUIRED)

set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")
set(SWIG_OUTFILE_DIR "${CMAKE_CURRENT_BINARY_DIR}")

if(${CMAKE_VERSION} VERSION_LESS "3.15.0")
  set(RL_Python_Module_Target "Python3::Python")
else()
  set(RL_Python_Module_Target "Python3::Module")
endif()

swig_add_library(roboticslab_vision_python
                 LANGUAGE python
                 SOURCES ${SWIG_BINDINGS_SOURCE_FILE})
swig_link_libraries(roboticslab_vision_python ${RL_Python_Module_Target} ${SWIG_RL_LIBRARIES})

set_target_properties(${SWIG_MODULE_roboticslab_vision_python_REAL_NAME} PROPERTIES OUTPUT_NAME "_roboticslab_vision"
                                                                                    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python"
                                                                                    # see target_include_directories() below
                                                                                    NO_SYSTEM_FROM_IMPORTED TRUE)

# The INCLUDE_DIRECTORIES directory property might get polluted by 3rd party dep headers, most probably VTK's and PCL's.
# Among those headers, there might be a /usr/lib/python2.x that predates Python3 development headers. The latter have
# less precedence because of the SYSTEM flag. We need to import them as non-SYSTEM (see NO_SYSTEM_FROM_IMPORTED above)
# and prepend to the existing list of header directories. CMake takes care of removing duplicates.
target_include_directories(${SWIG_MODULE_roboticslab_vision_python_REAL_NAME} BEFORE PRIVATE ${Python3_INCLUDE_DIRS})

# installation path is determined reliably on most platforms using distutils
execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(plat_specific=True,standard_lib=False,prefix=''))"
                OUTPUT_VARIABLE Python_INSTDIR
                OUTPUT_STRIP_TRAILING_WHITESPACE )

set(_CMAKE_INSTALL_PYTHONDIR "${Python_INSTDIR}")
set(CMAKE_INSTALL_PYTHONDIR ${_CMAKE_INSTALL_PYTHONDIR} CACHE PATH "python bindings (${_CMAKE_INSTALL_PYTHONDIR})")
mark_as_advanced(CMAKE_INSTALL_PYTHONDIR)
if(NOT IS_ABSOLUTE ${CMAKE_INSTALL_PYTHONDIR})
  set(CMAKE_INSTALL_FULL_PYTHONDIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_PYTHONDIR}")
else()
  set(CMAKE_INSTALL_FULL_PYTHONDIR "${CMAKE_INSTALL_PYTHONDIR}")
endif()

install(FILES ${CMAKE_BINARY_DIR}/lib/python/roboticslab_vision.py
        DESTINATION ${CMAKE_INSTALL_PYTHONDIR})

# Update RPATH
if(NOT CMAKE_SKIP_RPATH AND NOT CMAKE_SKIP_INSTALL_RPATH)
  file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_FULL_PYTHONDIR}" "${CMAKE_INSTALL_FULL_LIBDIR}")
  get_target_property(_current_rpath ${SWIG_MODULE_roboticslab_vision_python_REAL_NAME} INSTALL_RPATH)
  if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND _current_rpath "@loader_path/${_rel_path}")
  else()
    list(APPEND _current_rpath "\$ORIGIN/${_rel_path}")
  endif()
  set_target_properties(${SWIG_MODULE_roboticslab_vision_python_REAL_NAME} PROPERTIES INSTALL_RPATH "${_current_rpath}")
endif()

install(TARGETS ${SWIG_MODULE_roboticslab_vision_python_REAL_NAME}
        DESTINATION ${CMAKE_INSTALL_PYTHONDIR})
