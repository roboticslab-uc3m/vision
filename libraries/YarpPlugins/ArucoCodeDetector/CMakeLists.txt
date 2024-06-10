if(NOT DEFINED ENABLE_ArucoCodeDetector OR ENABLE_ArucoCodeDetector)
    if(NOT TARGET opencv_objdetect OR NOT OpenCV_VERSION VERSION_GREATER_EQUAL 4.7)
        message(WARNING "OpenCV objdetect module not found, disabling ArucoCodeDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling ArucoCodeDetector device")
    endif()
endif()

yarp_prepare_plugin(ArucoCodeDetector
                    CATEGORY device
                    TYPE roboticslab::ArucoCodeDetector
                    INCLUDE ArucoCodeDetector.hpp
                    DEFAULT ON
                    DEPENDS "TARGET opencv_objdetect;YARP_cv_FOUND")

if(NOT SKIP_ArucoCodeDetector)

    yarp_add_plugin(ArucoCodeDetector ArucoCodeDetector.hpp
                                      ArucoCodeDetector.cpp)

    target_link_libraries(ArucoCodeDetector YARP::YARP_os
                                            YARP::YARP_dev
                                            YARP::YARP_cv
                                            opencv_objdetect
                                            opencv_core
                                            ROBOTICSLAB::VisionInterfaces)

    yarp_install(TARGETS ArucoCodeDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_ArucoCodeDetector OFF CACHE BOOL "Enable/disable ArucoCodeDetector device" FORCE)

endif()
