if(NOT DEFINED ENABLE_DnnDetector OR ENABLE_DnnDetector)
    if(NOT TARGET opencv_dnn)
        message(WARNING "OpenCV dnn module not found, disabling DnnDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling DnnDetector device")
    endif()
endif()

yarp_prepare_plugin(DnnDetector
                    CATEGORY device
                    TYPE DnnDetector
                    INCLUDE DnnDetector.hpp
                    DEFAULT ON
                    DEPENDS "TARGET opencv_dnn;YARP_cv_FOUND"
                    GENERATE_PARSER)

if(NOT SKIP_DnnDetector)

    yarp_add_plugin(DnnDetector)

    target_sources(DnnDetector PRIVATE DnnDetector.hpp
                                       DnnDetector.cpp
                                       DnnDetector_ParamsParser.h
                                       DnnDetector_ParamsParser.cpp)

    target_link_libraries(DnnDetector YARP::YARP_os
                                      YARP::YARP_dev
                                      YARP::YARP_cv
                                      opencv_dnn
                                      ROBOTICSLAB::VisionInterfaces)

    yarp_install(TARGETS DnnDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_DnnDetector OFF CACHE BOOL "Enable/disable DnnDetector device" FORCE)

endif()
