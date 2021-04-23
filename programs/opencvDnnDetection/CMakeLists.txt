if(NOT DEFINED ENABLE_opencvDnnDetection OR ENABLE_opencvDnnDetection)
    if(NOT TARGET opencv_dnn OR NOT OpenCV_VERSION VERSION_GREATER_EQUAL 4.0)
        message(WARNING "OpenCV 4.0+/dnn module not found, disabling opencvDnnDetection program")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling opencvDnnDetection program")
    endif()
endif()

cmake_dependent_option(ENABLE_opencvDnnDetection "Enable/disable opencvDnnDetection program" ON
                       "TARGET opencv_dnn;OpenCV_VERSION VERSION_GREATER_EQUAL 4.0;YARP_cv_FOUND" OFF)

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
                                             opencv_dnn)

    install(TARGETS opencvDnnDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_opencvDnnDetection OFF CACHE BOOL "Enable/disable opencvDnnDetection program" FORCE)

endif()
