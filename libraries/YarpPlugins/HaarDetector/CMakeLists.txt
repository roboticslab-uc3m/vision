if(NOT DEFINED ENABLE_HaarDetector OR ENABLE_HaarDetector)
    if(NOT OpenCV_FOUND)
        message(WARNING "OpenCV package not found, disabling HaarDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling HaarDetector device")
    endif()
endif()

cmake_dependent_option(ENABLE_HaarDetector "Enable/disable HaarDetector" ON
                       "OpenCV_FOUND;YARP_cv_FOUND" OFF)

yarp_prepare_plugin(HaarDetector
                    CATEGORY device
                    TYPE roboticslab::HaarDetector
                    INCLUDE HaarDetector.hpp
                    DEFAULT ON)

if(NOT SKIP_HaarDetector)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(HaarDetector HaarDetector.hpp
                                 HaarDetector.cpp)

    target_link_libraries(HaarDetector YARP::YARP_os
                                       YARP::YARP_dev
                                       YARP::YARP_cv
                                       ${OpenCV_LIBS}
                                       ROBOTICSLAB::ColorDebug
                                       VisionInterfaces)

    target_compile_features(HaarDetector PUBLIC cxx_std_11)

    yarp_install(TARGETS HaarDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_HaarDetector OFF CACHE BOOL "Enable/disable HaarDetector device" FORCE)

endif()