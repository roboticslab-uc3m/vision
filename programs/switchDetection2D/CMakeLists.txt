if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_switchDetection2D OR ENABLE_switchDetection2D))
    message(WARNING "OpenCV package not found, disabling switchDetection2D program")
endif()

#if(NOT TensorflowCC_FOUND AND (NOT DEFINED ENABLE_switchDetection2D OR ENABLE_switchDetection2D))
#    message(WARNING "TensorflowCC package not found, disabling switchDetection2D program")
#endif()


#cmake_dependent_option(ENABLE_switchDetection2D "Enable/disable switchDetection2D program" ON
#                                                "ENABLE_TravisLib;OpenCV_FOUND;TensorflowCC_FOUND" OFF)

cmake_dependent_option(ENABLE_switchDetection2D "Enable/disable switchDetection2D program" ON
                                                "ENABLE_TravisLib;OpenCV_FOUND" OFF)

if(ENABLE_switchDetection2D)

    include_directories(${OpenCV_INCLUDE_DIRS})

    # ColorRegionDetection
    add_library(ColorRegionDetection Detector.hpp
                                     ColorRegionDetection2D.hpp
                                     ColorRegionDetection2D.cpp)

    target_link_libraries(ColorRegionDetection YARP::YARP_OS
                                               YARP::YARP_init
                                               YARP::YARP_dev
                                               YARP::YARP_sig
                                               ${OpenCV_LIBS}
                                               ROBOTICSLAB::ColorDebug
                                               TravisLib)

    # HaarDetection
    add_library(HaarDetection Detector.hpp
                              HaarDetector.hpp
                              HaarDetector.cpp)

    target_link_libraries(HaarDetection YARP::YARP_OS
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        ${OpenCV_LIBS}
                                        ROBOTICSLAB::ColorDebug)

    # TensorflowDetection
    #add_library(TensorflowDetection Detector.hpp
    #                                TensorflowDetection2D.hpp
    #                                TensorflowDetection2D.cpp
    #                                TensorflowDetector.hpp
    #                                TensorflowDetector.cpp)

    #target_link_libraries(TensorflowDetection YARP::YARP_OS
    #                                          YARP::YARP_init
    #                                          YARP::YARP_dev
    #                                          YARP::YARP_sig
    #                                          ROBOTICSLAB::ColorDebug
    #                                          TensorflowCC::Shared
    #                                          ${OpenCV_LIBS})

    # switchDetection2D
    add_executable(switchDetection2D main.cpp
                                     SwitchDetection2D.hpp
                                     SwitchDetection2D.cpp
                                     SegmentorThread.hpp
                                     SegmentorThread.cpp
                                     Detector.hpp
                                     Detector.cpp)

    target_link_libraries(switchDetection2D YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_dev
                                            YARP::YARP_sig
                                            ROBOTICSLAB::ColorDebug
                                            ColorRegionDetection
                                            HaarDetection)
                                            #TensorflowDetection)

    install(TARGETS switchDetection2D
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_switchDetection2D OFF CACHE BOOL "Enable/disable switchDetection2D program" FORCE)

endif()
