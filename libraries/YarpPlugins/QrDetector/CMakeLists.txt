include(CheckCXXSymbolExists)
set(CMAKE_REQUIRED_INCLUDES "${OpenCV_INCLUDE_DIRS}")
set(CMAKE_REQUIRED_QUIET TRUE)
check_cxx_symbol_exists(HAVE_QUIRC opencv2/cvconfig.h _have_quirc)

if(NOT DEFINED ENABLE_QrDetector OR ENABLE_QrDetector)
    if(NOT TARGET opencv_objdetect OR NOT (OpenCV_VERSION VERSION_GREATER_EQUAL 4.7 OR _have_quirc))
        message(WARNING "OpenCV 4.x objdetect module with QR support not found, disabling QrDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling QrDetector device")
    endif()
endif()

yarp_prepare_plugin(QrDetector
                    CATEGORY device
                    TYPE QrDetector
                    INCLUDE QrDetector.hpp
                    DEFAULT ON
                    DEPENDS "TARGET opencv_objdetect;OpenCV_VERSION VERSION_GREATER_EQUAL 4.7 OR _have_quirc;YARP_cv_FOUND"
                    GENERATE_PARSER)

if(NOT SKIP_QrDetector)

    yarp_add_plugin(QrDetector)

    target_sources(QrDetector PRIVATE QrDetector.hpp
                                      QrDetector.cpp
                                      QrDetector_ParamsParser.h
                                      QrDetector_ParamsParser.cpp)

    target_link_libraries(QrDetector YARP::YARP_os
                                     YARP::YARP_dev
                                     YARP::YARP_cv
                                     opencv_objdetect
                                     ROBOTICSLAB::VisionInterfaces)

    yarp_install(TARGETS QrDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_QrDetector OFF CACHE BOOL "Enable/disable QrDetector device" FORCE)

endif()
