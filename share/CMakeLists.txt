if(ENABLE_DnnDetector)
    yarp_install(FILES contexts/DnnDetector/DnnDetector.ini
                       contexts/DnnDetector/coco-object-categories.txt
                       contexts/DnnDetector/object_detection_classes_yolov3_sharon.txt
                 DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/DnnDetector)

    if(DOWNLOAD_MODELS)
        yarp_install(DIRECTORY ${CMAKE_SOURCE_DIR}/models/yolov3-tiny
                     DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/DnnDetector)
    endif()

    yarp_install(FILES contexts/teddy-bear.ppm
                 DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/DnnDetector/tests)
endif()

if(ENABLE_HaarDetector)
    yarp_install(FILES contexts/HaarDetector/haarcascade_cocacola_can.xml
                       contexts/HaarDetector/haarcascade_frontalface_alt.xml
                       contexts/HaarDetector/haarcascade_car.xml
                       contexts/HaarDetector/haarcascade_coke.xml
                       contexts/HaarDetector/haarcascade_lemon.xml
                       contexts/HaarDetector/haarcascade_sofa.xml
                       contexts/HaarDetector/haarcascade_chair.xml
                       contexts/HaarDetector/haarcascade_clock.xml
                       contexts/HaarDetector/haarcascade_cup.xml
                       contexts/HaarDetector/haarcascade_mouse.xml
                       contexts/HaarDetector/haarcascade_table.xml
                       contexts/HaarDetector/haarcascade_cutlery.xml
                       contexts/HaarDetector/haarcascade_keyboard.xml
                       contexts/HaarDetector/haarcascade_pal_logo.xml
                       contexts/HaarDetector/haarcascade_tv.xml
                       contexts/HaarDetector/haarcascade_dog.xml
                       contexts/HaarDetector/haarcascade_lamp.xml
                 DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/HaarDetector)

    if(DOWNLOAD_MODELS)
        yarp_install(DIRECTORY ${CMAKE_SOURCE_DIR}/models/lbfmodel
                     DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/HaarDetector)
    endif()

    yarp_install(FILES contexts/face-nc.pgm
                 DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/HaarDetector/tests)
endif()

if(ENABLE_DnnDetector)
endif()

yarp_install(FILES applications/ymanager.ini
                   applications/example_rgbDetection.xml
                   applications/example_rgbdDetection.xml
                   applications/visionFakeSensors.xml
                   applications/visionSensors.xml
                   applications/visionTools.xml
             DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_INSTALL_DIR})

add_subdirectory(QrDetector)
add_subdirectory(sceneReconstruction)
add_subdirectory(sensors)
