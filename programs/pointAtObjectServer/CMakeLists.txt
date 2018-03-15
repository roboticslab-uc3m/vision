find_package(PCL 1.6 QUIET)

if(NOT PCL_FOUND AND (NOT DEFINED ENABLE_pointAtObjectServer OR ENABLE_pointAtObjectServer))
    message(WARNING "PCL package not found, disabling pointAtObjectServer program")
endif()

cmake_dependent_option(ENABLE_pointAtObjectServer "Enable/disable pointAtObjectServer program" ON
                       PCL_FOUND OFF)

if(ENABLE_pointAtObjectServer)

    find_package(YARP REQUIRED)

    include_directories(${YARP_INCLUDE_DIRS}
                        ${PCL_INCLUDE_DIRS})

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

    target_compile_definitions(pointAtObjectServer PUBLIC ${PCL_DEFINITIONS})

    target_link_libraries(pointAtObjectServer YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_sig
                                              ${PCL_LIBRARIES})

    install(TARGETS pointAtObjectServer
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_pointAtObjectServer OFF CACHE BOOL "Enable/disable pointAtObjectServer program" FORCE)

endif()
