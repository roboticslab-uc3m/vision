# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(teo-head)

yarp_install(FILES ymanager.ini DESTINATION ${TEO-HEAD_DATA_INSTALL_DIR})  # Cleaner than ${TEO_HEAD_ROBOTS_INSTALL_DIR}/../ or similar.

### Go through single applications

add_subdirectory(cv1)
add_subdirectory(cvFaces)
add_subdirectory(speechRecognition)

