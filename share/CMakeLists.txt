if(ENABLE_DnnDetector)
    yarp_install(DIRECTORY contexts/DnnDetector
                 DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}
                 PATTERN tests EXCLUDE)

    if(DOWNLOAD_MODELS)
        yarp_install(DIRECTORY ${CMAKE_SOURCE_DIR}/models/yolov3-tiny
                     DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/DnnDetector)
    endif()

    if(ENABLE_tests)
        yarp_install(DIRECTORY contexts/DnnDetector/tests
                     DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/DnnDetector)
    endif()
endif()

if(ENABLE_HaarDetector)
    yarp_install(DIRECTORY contexts/HaarDetector
                 DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}
                 PATTERN tests EXCLUDE)

    if(DOWNLOAD_MODELS)
        yarp_install(DIRECTORY ${CMAKE_SOURCE_DIR}/models/lbfmodel
                     DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/HaarDetector)
    endif()

    if(ENABLE_tests)
        yarp_install(DIRECTORY contexts/HaarDetector/tests
                     DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/HaarDetector)
    endif()
endif()

if(ENABLE_QrDetector)
    if(ENABLE_tests)
        yarp_install(DIRECTORY contexts/QrDetector/tests
                     DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/QrDetector)
    endif()
endif()

if(ENABLE_ArucoDetector)
    if(ENABLE_tests)
        yarp_install(DIRECTORY contexts/ArucoDetector/tests
                     DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/ArucoDetector)
    endif()
endif()

if(ENABLE_sceneReconstruction)
    yarp_install(DIRECTORY contexts/sceneReconstruction
                 DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR})
endif()

yarp_install(DIRECTORY contexts/sensors
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR})

yarp_install(FILES applications/visionFakeSensors.xml
                   applications/visionSensors.xml
                   applications/visionTools.xml
                   applications/ymanager.ini
             DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_INSTALL_DIR})

if(ENABLE_rgbdDetection)
    yarp_install(FILES applications/example_rgbdDetection.xml
                 DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_INSTALL_DIR})
endif()

if(ENABLE_rgbDetection)
    yarp_install(FILES applications/example_rgbDetection.xml
                 DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_INSTALL_DIR})
endif()
