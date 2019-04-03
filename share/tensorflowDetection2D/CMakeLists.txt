yarp_install(FILES conf/tensorflowDetection.ini
                   conf/tensorflowDetection2D.ini
                   conf/labels_map.pbtxt
                   conf/frozen_inference_graph.pb
  
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/tensorflowDetection)

