# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES conf/haarcascade_cocacola_can.xml
                   conf/haarcascade_frontalface_alt.xml
                   conf/haarcascade_car.xml  
                   conf/haarcascade_coke.xml       
                   conf/haarcascade_lemon.xml  
                   conf/haarcascade_sofa.xml
                   conf/haarcascade_chair.xml  
                   conf/haarcascade_clock.xml    
                   conf/haarcascade_cup.xml  
                   conf/haarcascade_mouse.xml  
                   conf/haarcascade_table.xml  
                   conf/haarcascade_cutlery.xml  
                   conf/haarcascade_keyboard.xml 
                   conf/haarcascade_pal_logo.xml   
                   conf/haarcascade_tv.xml  
                   conf/haarcascade_dog.xml  
                   conf/haarcascade_lamp.xml  
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/HaarDetector)

yarp_install(FILES tests/face-nc.pgm
             DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR}/HaarDetector/tests)
