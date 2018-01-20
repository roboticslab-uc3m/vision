option(ENABLE_multipleDump "Enable/disable multipleDump program" ON)

if(ENABLE_multipleDump)

find_package(YARP REQUIRED)

get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_VISION_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_includes})

add_executable(multipleDump main.cpp
                            MultipleDump.hpp
                            MultipleDump.cpp
                            MultipleDumpThread.hpp
                            MultipleDumpThread.cpp)

target_link_libraries(multipleDump YARP::YARP_OS
                                   YARP::YARP_init)

install(TARGETS multipleDump
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
