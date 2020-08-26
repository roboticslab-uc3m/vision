if(NOT PCL_FOUND AND (NOT DEFINED ENABLE_pointAtObjectServer OR ENABLE_pointAtObjectServer))
    message(WARNING "PCL package not found, disabling pointAtObjectServer program")
endif()

cmake_dependent_option(ENABLE_pointAtObjectServer "Enable/disable pointAtObjectServer program" ON
                       PCL_FOUND OFF)

if(ENABLE_pointAtObjectServer)

    include_directories(${PCL_INCLUDE_DIRS})

    link_directories(${PCL_LIBRARY_DIRS})

    add_executable(pointAtObjectServer main.cpp
                                       PointAtObjectServer.hpp
                                       PointAtObjectServer.cpp
                                       LineCallbackPort.hpp
                                       LineCallbackPort.cpp
                                       SharedArea.hpp
                                       SharedArea.cpp
                                       vtkTimerCallback.hpp
                                       vtkTimerCallback.cpp)

    # workaround for PCL 1.8 (Ubuntu 18.04), exclude empty/blank strings as compile defs
    list(FILTER PCL_DEFINITIONS EXCLUDE REGEX "^ +$")
    target_compile_definitions(pointAtObjectServer PUBLIC ${PCL_DEFINITIONS})

    target_link_libraries(pointAtObjectServer YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_sig
                                              ${PCL_LIBRARIES})

    install(TARGETS pointAtObjectServer
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_pointAtObjectServer OFF CACHE BOOL "Enable/disable pointAtObjectServer program" FORCE)

endif()
