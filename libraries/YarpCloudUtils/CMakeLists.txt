option(ENABLE_YarpCloudUtils "Enable/disable YarpCloudUtils" ON)

if(ENABLE_YarpCloudUtils)

    add_library(YarpCloudUtils SHARED YarpCloudUtils.hpp
                                      YarpCloudUtils-inst.hpp)

    set_target_properties(YarpCloudUtils PROPERTIES PUBLIC_HEADER YarpCloudUtils.hpp)

    target_link_libraries(YarpCloudUtils PUBLIC YARP::YARP_sig)

    target_include_directories(YarpCloudUtils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                     $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS YarpCloudUtils
            EXPORT ROBOTICSLAB_VISION
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    set_property(GLOBAL APPEND PROPERTY _exported_targets YarpCloudUtils)
    set_property(GLOBAL APPEND PROPERTY _exported_dependencies YARP_sig)

    if(PCL_FOUND AND YARP_pcl_FOUND)
        target_sources(YarpCloudUtils PRIVATE YarpCloudUtils-pcl.cpp)
        target_link_libraries(YarpCloudUtils PRIVATE YARP::YARP_pcl ${PCL_LIBRARIES})
        set_property(GLOBAL APPEND PROPERTY _exported_dependencies PCL)
    else()
        target_sources(YarpCloudUtils PRIVATE YarpCloudUtils-tinyply.cpp tinyply.h)
        target_link_libraries(YarpCloudUtils PRIVATE YARP::YARP_os)
        set_property(SOURCE YarpCloudUtils-tinyply.cpp PROPERTY COMPILE_DEFINITIONS TINYPLY_IMPLEMENTATION)
    endif()

else()

    set(ENABLE_YarpCloudUtils OFF CACHE BOOL "Enable/disable YarpCloudUtils" FORCE)

endif()
