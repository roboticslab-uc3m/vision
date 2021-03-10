option(ENABLE_opencvDnnObjectDetection2D "Enable/disable opencvDnnObjectDetection2D program" ON)

if(ENABLE_opencvDnnObjectDetection2D)

    add_executable(opencvDnnObjectDetection2D main.cpp
                                 OpencvDnnObjectDetection2D.hpp
                                 OpencvDnnObjectDetection2D.cpp
                                 SegmentorThread.hpp
                                 SegmentorThread.cpp)

    target_link_libraries(opencvDnnObjectDetection2D YARP::YARP_os
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        ROBOTICSLAB::VisionInterfaces
                                        ${OpenCV_LIBRARIES})

    install(TARGETS opencvDnnObjectDetection2D
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_opencvDnnObjectDetection2D OFF CACHE BOOL "Enable/disable opencvDnnObjectDetection2D program" FORCE)

endif()



