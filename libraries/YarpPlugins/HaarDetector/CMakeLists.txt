if(NOT DEFINED ENABLE_HaarDetector OR ENABLE_HaarDetector)
    if(NOT TARGET opencv_objdetect)
        message(WARNING "OpenCV objdetect module not found, disabling HaarDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling HaarDetector device")
    endif()
endif()

yarp_prepare_plugin(HaarDetector
                    CATEGORY device
                    TYPE roboticslab::HaarDetector
                    INCLUDE HaarDetector.hpp
                    DEFAULT ON
                    DEPENDS "TARGET opencv_objdetect;YARP_cv_FOUND")

if(NOT SKIP_HaarDetector)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(HaarDetector HaarDetector.hpp
                                 HaarDetector.cpp)

    target_link_libraries(HaarDetector YARP::YARP_os
                                       YARP::YARP_dev
                                       YARP::YARP_cv
                                       opencv_objdetect
                                       ROBOTICSLAB::VisionInterfaces)

    yarp_install(TARGETS HaarDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_HaarDetector OFF CACHE BOOL "Enable/disable HaarDetector device" FORCE)

endif()
