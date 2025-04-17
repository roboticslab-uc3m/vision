if(NOT DEFINED ENABLE_ArucoDetector OR ENABLE_ArucoDetector)
    if(NOT TARGET opencv_objdetect OR NOT OpenCV_VERSION VERSION_GREATER_EQUAL 4.7)
        message(WARNING "OpenCV objdetect module not found, disabling ArucoDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling ArucoDetector device")
    endif()
endif()

yarp_prepare_plugin(ArucoDetector
                    CATEGORY device
                    TYPE ArucoDetector
                    INCLUDE ArucoDetector.hpp
                    DEFAULT ON
                    DEPENDS "TARGET opencv_objdetect;OpenCV_VERSION VERSION_GREATER_EQUAL 4.7;YARP_cv_FOUND"
                    GENERATE_PARSER)

if(NOT SKIP_ArucoDetector)

    yarp_add_plugin(ArucoDetector)

    target_sources(ArucoDetector PRIVATE ArucoDetector.hpp
                                         ArucoDetector.cpp
                                         ArucoDetector_ParamsParser.h
                                         ArucoDetector_ParamsParser.cpp)

    target_link_libraries(ArucoDetector YARP::YARP_os
                                        YARP::YARP_dev
                                        YARP::YARP_cv
                                        opencv_objdetect
                                        opencv_core
                                        ROBOTICSLAB::VisionInterfaces)

    yarp_install(TARGETS ArucoDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_ArucoDetector OFF CACHE BOOL "Enable/disable ArucoDetector device" FORCE)

endif()
