# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES conf/colorRegionDetection.ini
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/colorRegionDetection)

yarp_install(FILES scripts/example_colorRegionDetection.xml
                   scripts/example_colorRegionDetection_local.xml
             DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_TEMPLATES_INSTALL_DIR})
