option(ENABLE_TravisLib "Enable/disable TravisLib" ON)

if(ENABLE_TravisLib)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(TravisLib TravisLib.hpp
                      TravisLib.cpp)

set_target_properties(TravisLib PROPERTIES PUBLIC_HEADER TravisLib.hpp)

target_link_libraries(TravisLib PUBLIC ${OpenCV_LIBS})

target_include_directories(TravisLib PUBLIC ${OpenCV_INCLUDE_DIRS}
                                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS TravisLib
        EXPORT ROBOTICSLAB_VISION
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_VISION_TARGETS TravisLib)

endif()
