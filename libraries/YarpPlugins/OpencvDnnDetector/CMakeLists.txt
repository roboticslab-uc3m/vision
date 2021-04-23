if(NOT DEFINED ENABLE_OpencvDnnDetector OR ENABLE_OpencvDnnDetector)
    if(NOT OpenCV_FOUND OR NOT OpenCV_VERSION_MAJOR EQUAL 4)
        message(WARNING "OpenCV 4.x package not found, disabling OpencvDnnDetector device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling OpencvDnnDetector device")
    endif()
endif()

yarp_prepare_plugin(OpencvDnnDetector
                    CATEGORY device
                    TYPE roboticslab::OpencvDnnDetector
                    INCLUDE OpencvDnnDetector.hpp
                    DEFAULT ON
                    DEPENDS "OpenCV_VERSION_MAJOR EQUAL 4;YARP_cv_FOUND")

if(NOT SKIP_OpencvDnnDetector)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(OpencvDnnDetector OpencvDnnDetector.hpp
                                      OpencvDnnDetector.cpp)

    target_link_libraries(OpencvDnnDetector YARP::YARP_os
                                            YARP::YARP_dev
                                            YARP::YARP_cv
                                            ${OpenCV_LIBRARIES}
                                            ROBOTICSLAB::VisionInterfaces
                                            ROBOTICSLAB::TravisLib)

    target_compile_features(OpencvDnnDetector PUBLIC cxx_std_11)

    yarp_install(TARGETS OpencvDnnDetector
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_OpencvDnnDetector OFF CACHE BOOL "Enable/disable OpencvDnnDetector device" FORCE)

endif()
