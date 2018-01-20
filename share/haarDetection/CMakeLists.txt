# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES conf/haarDetection.ini
                   conf/haarDetection2D.ini
                   conf/haarcascade_cocacola_can.xml
                   conf/haarcascade_frontalface_alt.xml
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/haarDetection)

yarp_install(FILES scripts/example_haarDetection.xml
                   scripts/example_haarDetection2D.xml
             DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_TEMPLATES_INSTALL_DIR})
