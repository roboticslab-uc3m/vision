if(NOT YARP_idl_tools_FOUND AND (NOT DEFINED ENABLE_VisionIDL OR ENABLE_VisionIDL))
    message(WARNING "YARP idl_tools component not found, disabling VisionIDL library")
endif()

cmake_dependent_option(ENABLE_VisionIDL "Enable/disable VisionIDL library" ON
                       YARP_idl_tools_FOUND OFF)

if(ENABLE_VisionIDL)

    set(ALLOW_IDL_GENERATION ON CACHE BOOL "Detect changes and rebuild IDL files")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS vision.thrift)

    yarp_idl_to_dir(INPUT_FILES vision.thrift
                    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}
                    SOURCES_VAR idl_sources
                    HEADERS_VAR idl_headers
                    INCLUDE_DIRS_VAR idl_include_dirs
                    PLACEMENT MERGED
                    THRIFT_NO_NAMESPACE_PREFIX)

    add_library(VisionIDL SHARED ${idl_sources} ${idl_headers})

    set_target_properties(VisionIDL PROPERTIES PUBLIC_HEADER "${idl_headers}")

    target_link_libraries(VisionIDL PUBLIC YARP::YARP_os
                                           YARP::YARP_sig)

    target_include_directories(VisionIDL PUBLIC $<BUILD_INTERFACE:${idl_include_dirs}>
                                                $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS VisionIDL
            EXPORT ROBOTICSLAB_SPEECH
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    set_property(GLOBAL APPEND PROPERTY _exported_targets VisionIDL)

    add_library(ROBOTICSLAB::VisionIDL ALIAS VisionIDL)

else()

    set(ENABLE_VisionIDL OFF CACHE BOOL "Enable/disable VisionIDL library" FORCE)

endif()
