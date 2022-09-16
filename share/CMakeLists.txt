# Copyright: 2019 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU LGPL v2.1.

yarp_install(FILES applications/ymanager.ini
                   applications/example_rgbDetection.xml
                   applications/example_rgbdDetection.xml
                   applications/visionFakeSensors.xml
                   applications/visionSensors.xml
                   applications/visionTools.xml
             DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_INSTALL_DIR})

### Go through single applications

if(ENABLE_DnnDetector)
    add_subdirectory(DnnDetector)
endif()

if(ENABLE_HaarDetector)
    add_subdirectory(HaarDetector)
endif()

if(ENABLE_QrDetector)
    add_subdirectory(QrDetector)
endif()

if(ENABLE_sceneReconstruction)
    add_subdirectory(sceneReconstruction)
endif()

add_subdirectory(sensors)
