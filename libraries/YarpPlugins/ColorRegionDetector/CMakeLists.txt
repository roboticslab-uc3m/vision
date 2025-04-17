if(NOT YARP_cv_FOUND AND (NOT DEFINED ENABLE_ColorRegionDetector OR ENABLE_ColorRegionDetector))
    message(WARNING "YARP_cv package not found, disabling ColorRegionDetector device")
endif()

yarp_prepare_plugin(ColorRegionDetector
                    CATEGORY device
                    TYPE ColorRegionDetector
                    INCLUDE ColorRegionDetector.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_TravisLib;YARP_cv_FOUND"
                    GENERATE_PARSER)

if(NOT SKIP_ColorRegionDetector)

    yarp_add_plugin(ColorRegionDetector)

    target_sources(ColorRegionDetector PRIVATE ColorRegionDetector.hpp
                                               ColorRegionDetector.cpp
                                               ColorRegionDetector_ParamsParser.h
                                               ColorRegionDetector_ParamsParser.cpp)

    target_link_libraries(ColorRegionDetector YARP::YARP_os
                                              YARP::YARP_dev
                                              YARP::YARP_cv
                                              ROBOTICSLAB::VisionInterfaces
                                              ROBOTICSLAB::TravisLib)

    yarp_install(TARGETS ColorRegionDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_ColorRegionDetector OFF CACHE BOOL "Enable/disable ColorRegionDetector device" FORCE)

endif()
