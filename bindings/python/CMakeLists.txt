# Created by Juan G Victores 2013 @ UC3M
# Thanks to Paul Fitzpatrick for all the YARP and iCub SWIG stuff for inspiration

# this is set in the parent directory, but for some reason it does not get inherited
set_source_files_properties(${SWIG_BINDINGS_SOURCE_FILE} PROPERTIES CPLUSPLUS ON)

set(CMAKE_SWIG_FLAGS "-Wall;-module;roboticslab_vision;-threads")
set(CREATE_PYTHON_VERSION "" CACHE STRING "Control python version used, if you wish." )

find_package(PythonInterp ${CREATE_PYTHON_VERSION})

find_package(PythonLibs ${CREATE_PYTHON_VERSION})
include_directories(${PYTHON_INCLUDE_PATH})

link_libraries(${PYTHON_LIBRARY})
  
## in Windows it seems necessary to declare explictly the link_directory
if (WIN32)
  get_filename_component(PYTHON_DIR ${PYTHON_LIBRARY} PATH)
  link_directories(${PYTHON_DIR})
endif()

set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")
  
swig_add_library(roboticslab_vision
                 LANGUAGE python
                 SOURCES ${SWIG_BINDINGS_SOURCE_FILE})
#swig_link_libraries(roboticslab_vision ${PYTHON_LIBRARY} VisionInterfaces)
#target_include_directories(roboticslab_vision SYSTEM PRIVATE ${PYTHON_INCLUDE_PATH})
swig_link_libraries(roboticslab_vision ${PYTHON_LIBRARY}
                                       YARP::YARP_os
                                       YARP::YARP_sig)


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
