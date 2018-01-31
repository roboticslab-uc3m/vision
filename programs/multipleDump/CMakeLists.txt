option(ENABLE_multipleDump "Enable/disable multipleDump program" ON)

if(ENABLE_multipleDump)

find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})

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
