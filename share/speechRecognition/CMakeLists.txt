# Copyright: 2016 UC3M
# Authors: Juan G Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname speechRecognition)


file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB lm_spanish ${CMAKE_CURRENT_SOURCE_DIR}/conf/dictionary/spanish/*.lm)
file(GLOB dic_spanish ${CMAKE_CURRENT_SOURCE_DIR}/conf/dictionary/spanish/*.dic)
file(GLOB es_model ${CMAKE_CURRENT_SOURCE_DIR}/conf/model/es/*)


yarp_install(FILES ${conf} DESTINATION ${TEO-HEAD_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${lm_spanish} DESTINATION ${TEO-HEAD_CONTEXTS_INSTALL_DIR}/${appname}/dictionary/spanish)
yarp_install(FILES ${dic_spanish} DESTINATION ${TEO-HEAD_CONTEXTS_INSTALL_DIR}/${appname}/dictionary/spanish)
yarp_install(FILES ${es_model} DESTINATION ${TEO-HEAD_CONTEXTS_INSTALL_DIR}/${appname}/model/es)

