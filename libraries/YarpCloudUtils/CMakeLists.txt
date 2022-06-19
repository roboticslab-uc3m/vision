option(ENABLE_YarpCloudUtils "Enable/disable YarpCloudUtils" ON)

if(ENABLE_YarpCloudUtils)

    if(PCL_VERSION VERSION_LESS 1.9)
        include_directories(${PCL_INCLUDE_DIRS})
        link_directories(${PCL_LIBRARY_DIRS})

        # workaround for PCL 1.8 (Ubuntu 18.04), exclude empty/blank strings as compile defs
        list(FILTER PCL_DEFINITIONS EXCLUDE REGEX "^ +$")
        add_definitions(${PCL_DEFINITIONS})
    endif()

    add_library(YarpCloudUtils SHARED YarpCloudUtils.hpp
                                      YarpCloudUtils-ply-export.cpp
                                      YarpCloudUtils-ply-import.cpp
                                      YarpCloudUtils-pcl.cpp
                                      tinyply.h
                                      LogComponent.hpp
                                      LogComponent.cpp)

    set_property(TARGET YarpCloudUtils PROPERTY PUBLIC_HEADER YarpCloudUtils.hpp)

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/tinyply.cpp "#include \"tinyply.h\"")
    set_property(SOURCE ${CMAKE_CURRENT_BINARY_DIR}/tinyply.cpp PROPERTY COMPILE_DEFINITIONS TINYPLY_IMPLEMENTATION)
    target_sources(YarpCloudUtils PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/tinyply.cpp)

    target_link_libraries(YarpCloudUtils PUBLIC YARP::YARP_os
                                                YARP::YARP_sig)

    target_include_directories(YarpCloudUtils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                     $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS YarpCloudUtils
            EXPORT ROBOTICSLAB_VISION)

    set_property(GLOBAL APPEND PROPERTY _exported_targets YarpCloudUtils)
    set_property(GLOBAL APPEND PROPERTY _exported_dependencies YARP_os YARP_sig)

    if(PCL_FOUND AND YARP_pcl_FOUND)
        target_sources(YarpCloudUtils PRIVATE YarpCloudUtils-pcl.hpp
                                              YarpCloudUtils-pcl-traits.hpp
                                              YarpCloudUtils-pcl-impl.hpp
                                              YarpCloudUtils-pcl-inst.hpp)

        target_link_libraries(YarpCloudUtils PRIVATE YARP::YARP_pcl
                                                     ${PCL_LIBRARIES})

        target_compile_definitions(YarpCloudUtils PUBLIC YCU_HAVE_PCL)
    endif()

    add_library(ROBOTICSLAB::YarpCloudUtils ALIAS YarpCloudUtils)

else()

    set(ENABLE_YarpCloudUtils OFF CACHE BOOL "Enable/disable YarpCloudUtils" FORCE)

endif()
