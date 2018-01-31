# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(roboticslab-vision)

yarp_install(FILES ymanager.ini
             DESTINATION ${ROBOTICSLAB-VISION_DATA_INSTALL_DIR})

### Go through single applications

add_subdirectory(colorRegionDetection)
add_subdirectory(haarDetection)
#add_subdirectory(pointAtObject)  # never had a cmake
