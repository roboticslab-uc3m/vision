option(ENABLE_YarpCloudUtils "Enable/disable YarpCloudUtils" ON)

if(ENABLE_YarpCloudUtils)

    add_library(YarpCloudUtils SHARED YarpCloudUtils.hpp
                                      YarpCloudUtils-ply-export.cpp
                                      YarpCloudUtils-ply-import.cpp
                                      YarpCloudUtils-pcl.cpp
                                      tinyply.h)

    set_property(TARGET YarpCloudUtils PROPERTY PUBLIC_HEADER YarpCloudUtils.hpp)

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/tinyply.cpp "#include \"tinyply.h\"")
    set_property(SOURCE ${CMAKE_CURRENT_BINARY_DIR}/tinyply.cpp PROPERTY COMPILE_DEFINITIONS TINYPLY_IMPLEMENTATION)
    target_sources(YarpCloudUtils PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/tinyply.cpp)

    target_link_libraries(YarpCloudUtils PUBLIC YARP::YARP_os
                                                YARP::YARP_sig)

    target_include_directories(YarpCloudUtils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                     $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS YarpCloudUtils
            EXPORT ROBOTICSLAB_VISION
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    set_property(GLOBAL APPEND PROPERTY _exported_targets YarpCloudUtils)
    set_property(GLOBAL APPEND PROPERTY _exported_dependencies YARP_os YARP_sig)

    if(PCL_FOUND AND YARP_pcl_FOUND)
        set_property(SOURCE YarpCloudUtils-pcl.cpp PROPERTY COMPILE_DEFINITIONS HAVE_PCL)

        target_sources(YarpCloudUtils PRIVATE YarpCloudUtils-pcl.hpp
                                              YarpCloudUtils-pcl-traits.hpp
                                              YarpCloudUtils-pcl-impl.hpp
                                              YarpCloudUtils-pcl-inst.hpp)

        target_link_libraries(YarpCloudUtils PRIVATE YARP::YARP_pcl
                                                     ${PCL_LIBRARIES})
    endif()

else()

    set(ENABLE_YarpCloudUtils OFF CACHE BOOL "Enable/disable YarpCloudUtils" FORCE)

endif()
