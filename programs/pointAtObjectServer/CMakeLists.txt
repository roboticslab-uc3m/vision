if(NOT PCL_FOUND AND (NOT DEFINED ENABLE_pointAtObjectServer OR ENABLE_pointAtObjectServer))
    message(WARNING "PCL package not found, disabling pointAtObjectServer program")
endif()

cmake_dependent_option(ENABLE_pointAtObjectServer "Enable/disable pointAtObjectServer program" ON
                       PCL_FOUND OFF)

if(ENABLE_pointAtObjectServer)

    if(PCL_VERSION VERSION_LESS 1.9)
        include_directories(${PCL_INCLUDE_DIRS})
        link_directories(${PCL_LIBRARY_DIRS})

        # workaround for PCL 1.8 (Ubuntu 18.04), exclude empty/blank strings as compile defs
        list(FILTER PCL_DEFINITIONS EXCLUDE REGEX "^ +$")
        add_definitions(${PCL_DEFINITIONS})
    endif()

    add_executable(pointAtObjectServer main.cpp
                                       PointAtObjectServer.hpp
                                       PointAtObjectServer.cpp
                                       LineCallbackPort.hpp
                                       LineCallbackPort.cpp
                                       SharedArea.hpp
                                       SharedArea.cpp
                                       vtkTimerCallback.hpp
                                       vtkTimerCallback.cpp)

    target_link_libraries(pointAtObjectServer YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_sig
                                              ${PCL_LIBRARIES})

    install(TARGETS pointAtObjectServer)

else()

    set(ENABLE_pointAtObjectServer OFF CACHE BOOL "Enable/disable pointAtObjectServer program" FORCE)

endif()
