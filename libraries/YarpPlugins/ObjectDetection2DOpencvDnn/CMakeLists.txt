if(NOT DEFINED ENABLE_ObjectDetection2DOpencvDnn OR ENABLE_ObjectDetection2DOpencvDnn)
    if(NOT OpenCV_FOUND OR NOT OpenCV_VERSION_MAJOR EQUAL 4)
        message(WARNING "OpenCV 4.x package not found, disabling ObjectDetection2DOpencvDnn device")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling ObjectDetection2DOpencvDnn device")
    endif()
endif()

yarp_prepare_plugin(ObjectDetection2DOpencvDnn
                    CATEGORY device
                    TYPE roboticslab::ObjectDetection2DOpencvDnn
                    INCLUDE ObjectDetection2DOpencvDnn.hpp
                    DEFAULT ON
                    DEPENDS "OpenCV_VERSION_MAJOR EQUAL 4;YARP_cv_FOUND")

if(NOT SKIP_ObjectDetection2DOpencvDnn)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(ObjectDetection2DOpencvDnn ObjectDetection2DOpencvDnn.hpp
                               ObjectDetection2DOpencvDnn.cpp)

    target_link_libraries(ObjectDetection2DOpencvDnn YARP::YARP_os
                                     YARP::YARP_dev
                                     YARP::YARP_cv
                                     ${OpenCV_LIBRARIES}
                                     ROBOTICSLAB::VisionInterfaces
                                     ROBOTICSLAB::TravisLib)

    target_compile_features(ObjectDetection2DOpencvDnn PUBLIC cxx_std_11)

    yarp_install(TARGETS ObjectDetection2DOpencvDnn
                 LIBRARY DESTINATION ${ROBOTICSLAB-VISION_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-VISION_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-VISION_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_ObjectDetection2DOpencvDnn OFF CACHE BOOL "Enable/disable ObjectDetection2DOpencvDnn device" FORCE)

endif()
