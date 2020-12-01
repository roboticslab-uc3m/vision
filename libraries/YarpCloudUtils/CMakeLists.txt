option(ENABLE_YarpCloudUtils "Enable/disable YarpCloudUtils" ON)

if(ENABLE_YarpCloudUtils)

    add_library(YarpCloudUtils SHARED YarpCloudUtils.hpp
                                      YarpCloudUtils-tinyply.cpp
                                      YarpCloudUtils-pcl.cpp
                                      YarpCloudUtils-traits.hpp
                                      tinyply.h)

    set_property(TARGET YarpCloudUtils PROPERTY PUBLIC_HEADER YarpCloudUtils.hpp)

    set_property(SOURCE YarpCloudUtils-tinyply.cpp PROPERTY COMPILE_DEFINITIONS TINYPLY_IMPLEMENTATION)

    target_link_libraries(YarpCloudUtils PUBLIC YARP::YARP_os YARP::YARP_sig)

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
        target_link_libraries(YarpCloudUtils PRIVATE YARP::YARP_pcl ${PCL_LIBRARIES})
    endif()

else()

    set(ENABLE_YarpCloudUtils OFF CACHE BOOL "Enable/disable YarpCloudUtils" FORCE)

endif()
