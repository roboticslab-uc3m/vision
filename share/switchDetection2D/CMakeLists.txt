# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES conf/switchDetection2D.ini
                   conf/switchDetection2D.ini
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/switchDetection2D)

yarp_install(FILES scripts/example_switchDetection.xml
             DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_INSTALL_DIR})
