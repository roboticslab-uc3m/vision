if(NOT YARP_cv_FOUND AND (NOT DEFINED ENABLE_ColorRegionDetector OR ENABLE_ColorRegionDetector))
    message(WARNING "YARP_cv package not found, disabling ColorRegionDetector device")
endif()

yarp_prepare_plugin(ColorRegionDetector
                    CATEGORY device
                    TYPE roboticslab::ColorRegionDetector
                    INCLUDE ColorRegionDetector.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_TravisLib;YARP_cv_FOUND")

if(NOT SKIP_ColorRegionDetector)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(ColorRegionDetector ColorRegionDetector.hpp
                                        ColorRegionDetector.cpp)

    target_link_libraries(ColorRegionDetector YARP::YARP_os
                                              YARP::YARP_dev
                                              YARP::YARP_cv
                                              ROBOTICSLAB::ColorDebug
                                              VisionInterfaces
                                              TravisLib)

    target_compile_features(ColorRegionDetector PUBLIC cxx_std_11)

    yarp_install(TARGETS ColorRegionDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_ColorRegionDetector OFF CACHE BOOL "Enable/disable ColorRegionDetector device" FORCE)

endif()
