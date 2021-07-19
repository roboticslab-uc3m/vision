include(CheckCXXSymbolExists)
set(CMAKE_REQUIRED_INCLUDES "${OpenCV_INCLUDE_DIRS}")
set(CMAKE_REQUIRED_QUIET TRUE)
check_cxx_symbol_exists(HAVE_QUIRC opencv2/cvconfig.h _have_quirc)

if(NOT DEFINED ENABLE_QrDetector OR ENABLE_QrDetector)
    if(NOT TARGET opencv_imgproc OR NOT OpenCV_VERSION_MAJOR EQUAL 4 OR NOT _have_quirc)
        message(WARNING "OpenCV 4.x imgproc module with QUIRC support not found, disabling QrDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling QrDetector device")
    endif()
endif()

yarp_prepare_plugin(QrDetector
                    CATEGORY device
                    TYPE roboticslab::QrDetector
                    INCLUDE QrDetector.hpp
                    DEFAULT ON
                    DEPENDS "TARGET opencv_imgproc;OpenCV_VERSION_MAJOR EQUAL 4;_have_quirc;YARP_cv_FOUND")

if(NOT SKIP_QrDetector)

    yarp_add_plugin(QrDetector QrDetector.hpp
                               QrDetector.cpp)

    target_link_libraries(QrDetector YARP::YARP_os
                                     YARP::YARP_dev
                                     YARP::YARP_cv
                                     opencv_imgproc
                                     ROBOTICSLAB::VisionInterfaces)

    yarp_install(TARGETS QrDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_QrDetector OFF CACHE BOOL "Enable/disable QrDetector device" FORCE)

endif()
