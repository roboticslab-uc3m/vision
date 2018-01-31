option(ENABLE_pointAtObjectServer "Enable/disable pointAtObjectServer program" OFF)

if(ENABLE_pointAtObjectServer)

find_package(YARP REQUIRED)
find_package(PCL 1.6 REQUIRED)

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

endif()
