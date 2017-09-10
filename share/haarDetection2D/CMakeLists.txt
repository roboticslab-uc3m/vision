# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname haarDetection2D)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB xml ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.xml)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

yarp_install(FILES ${conf} DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${xml} DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scripts} DESTINATION ${ROBOTICSLAB-VISION_APPLICATIONS_TEMPLATES_INSTALL_DIR})
