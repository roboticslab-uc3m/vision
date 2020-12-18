if(NOT DEFINED ENABLE_QRDetector OR ENABLE_QRDetector)
    if(NOT OpenCV_FOUND OR NOT OpenCV_VERSION_MAJOR EQUAL 4)
        message(WARNING "OpenCV 4.x package not found, disabling QRDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling QRDetector device")
    endif()
endif()

yarp_prepare_plugin(QRDetector
                    CATEGORY device
                    TYPE roboticslab::QRDetector
                    INCLUDE QRDetector.hpp
                    DEFAULT ON
                    DEPENDS "OpenCV_VERSION_MAJOR EQUAL 4;YARP_cv_FOUND")

if(NOT SKIP_QRDetector)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(QRDetector QRDetector.hpp
                               QRDetector.cpp)

    target_link_libraries(QRDetector YARP::YARP_os
                                     YARP::YARP_dev
                                     YARP::YARP_cv
                                     ${OpenCV_LIBS}
                                     ROBOTICSLAB::VisionInterfaces)

    target_compile_features(QRDetector PUBLIC cxx_std_11)

    yarp_install(TARGETS QRDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_QRDetector OFF CACHE BOOL "Enable/disable QRDetector device" FORCE)

endif()
