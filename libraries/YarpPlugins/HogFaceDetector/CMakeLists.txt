if(NOT DEFINED ENABLE_HogFaceDetector OR ENABLE_HogFaceDetector)
    if(NOT TARGET opencv_imgproc)
        message(WARNING "OpenCV imgproc module not found, disabling HogFaceDetector device")
    elseif(NOT dlib_FOUND)
        message(WARNING "dlib package not found, disabling HogFaceDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling HogFaceDetector device")
    endif()
endif()

yarp_prepare_plugin(HogFaceDetector
                    CATEGORY device
                    TYPE roboticslab::HogFaceDetector
                    INCLUDE HogFaceDetector.hpp
                    DEFAULT ON
                    DEPENDS "TARGET opencv_imgproc;dlib_FOUND;YARP_cv_FOUND")

if(NOT SKIP_HogFaceDetector)

    yarp_add_plugin(HogFaceDetector HogFaceDetector.hpp
                                    HogFaceDetector.cpp)

    target_link_libraries(HogFaceDetector YARP::YARP_os
                                          YARP::YARP_dev
                                          YARP::YARP_cv
                                          opencv_imgproc
                                          dlib::dlib
                                          ROBOTICSLAB::VisionInterfaces)

    yarp_install(TARGETS HogFaceDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_HogFaceDetector OFF CACHE BOOL "Enable/disable HogFaceDetector device" FORCE)

endif()
