option(ENABLE_opencvDnnDetection "Enable/disable opencvDnnDetection program" ON)

if(ENABLE_opencvDnnDetection)

    add_executable(opencvDnnDetection main.cpp
                                      OpencvDnnDetection.hpp
                                      OpencvDnnDetection.cpp
                                      SegmentorThread.hpp
                                      SegmentorThread.cpp)

    target_link_libraries(opencvDnnDetection YARP::YARP_os
                                             YARP::YARP_init
                                             YARP::YARP_dev
                                             YARP::YARP_sig
                                             ROBOTICSLAB::VisionInterfaces
                                             ${OpenCV_LIBRARIES})

    install(TARGETS opencvDnnDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_opencvDnnDetection OFF CACHE BOOL "Enable/disable opencvDnnDetection program" FORCE)

endif()
