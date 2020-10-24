if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_TravisLib OR ENABLE_TravisLib))
    message(WARNING "OpenCV package not found, disabling TravisLib")
endif()

cmake_dependent_option(ENABLE_TravisLib "Enable/disable TravisLib" ON
                       OpenCV_FOUND OFF)

if(ENABLE_TravisLib)

    add_library(TravisLib SHARED TravisLib.hpp
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

    set_property(GLOBAL APPEND PROPERTY _exported_targets TravisLib)
    set_property(GLOBAL APPEND PROPERTY _exported_dependencies OpenCV)

else()

    set(ENABLE_TravisLib OFF CACHE BOOL "Enable/disable TravisLib" FORCE)

endif()
