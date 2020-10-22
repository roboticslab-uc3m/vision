# Copyright: 2019 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU LGPL v2.1.

yarp_install(FILES applications/ymanager.ini
                   applications/example_switchDetection2D.xml
                   applications/fakeSensors.xml
             DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_INSTALL_DIR})

### Go through single applications

add_subdirectory(colorRegionDetection)
add_subdirectory(switchDetection)
add_subdirectory(tensorflowDetection2D)
#add_subdirectory(pointAtObject)  # never had a cmake
add_subdirectory(sensors)
