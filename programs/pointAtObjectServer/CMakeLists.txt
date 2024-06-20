if(NOT PCL_FOUND AND (NOT DEFINED ENABLE_pointAtObjectServer OR ENABLE_pointAtObjectServer))
    message(WARNING "PCL package not found, disabling pointAtObjectServer program")
endif()

cmake_dependent_option(ENABLE_pointAtObjectServer "Enable/disable pointAtObjectServer program" ON
                       PCL_FOUND OFF)

if(ENABLE_pointAtObjectServer)

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
