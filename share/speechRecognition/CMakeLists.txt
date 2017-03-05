# Copyright: 2016 UC3M
# Authors: Juan G Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname speechRecognition)


file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB lm ${CMAKE_CURRENT_SOURCE_DIR}/conf/dictionary/*.lm)
file(GLOB dic ${CMAKE_CURRENT_SOURCE_DIR}/conf/dictionary/*.dic)
file(GLOB es_model ${CMAKE_CURRENT_SOURCE_DIR}/conf/model/es/*)
file(GLOB en_model ${CMAKE_CURRENT_SOURCE_DIR}/conf/model/en-us/*)

yarp_install(FILES ${conf} DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${lm} DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/${appname}/dictionary)
yarp_install(FILES ${dic} DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/${appname}/dictionary)
yarp_install(FILES ${es_model} DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/${appname}/model/es)
yarp_install(FILES ${en_model} DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/${appname}/model/en-us)

