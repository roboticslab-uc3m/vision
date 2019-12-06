if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_switchDetection2D OR ENABLE_switchDetection2D))
    message(WARNING "OpenCV package not found, disabling switchDetection2D program")
endif()

if(NOT TensorflowCC_FOUND AND (NOT DEFINED ENABLE_switchDetection2D OR ENABLE_switchDetection2D))
    message(WARNING "TensorflowCC package not found, disabling switchDetection2D program")
endif()


cmake_dependent_option(ENABLE_switchDetection2D "Enable/disable switchDetection2D program" ON
                                                "ENABLE_TravisLib;OpenCV_FOUND;TensorflowCC_FOUND" OFF)

if(ENABLE_switchDetection2D)

    include_directories(${OpenCV_INCLUDE_DIRS})

    # ColorRegionDetector
    add_library(ColorRegionDetector Detector.hpp
                                    ColorRegionDetector.hpp
                                    ColorRegionDetector.cpp)

    target_link_libraries(ColorRegionDetector YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig
                                              ${OpenCV_LIBS}
                                              ROBOTICSLAB::ColorDebug
                                              TravisLib) # ROBOTICSLAB::TravisLib ? ...only finds installed header!!

    # HaarDetector
    add_library(HaarDetector Detector.hpp
                             HaarDetector.hpp
                             HaarDetector.cpp)

    target_link_libraries(HaarDetector YARP::YARP_OS
                                       YARP::YARP_init
                                       YARP::YARP_dev
                                       YARP::YARP_sig
                                       ${OpenCV_LIBS}
                                       ROBOTICSLAB::ColorDebug)

    # TensorflowDetector
    add_library(TensorflowDetector Detector.hpp
                                   TensorFlowDetector.hpp
                                   TensorFlowDetector.cpp
                                   TensorFlowDetectorHelper.hpp
                                   TensorFlowDetectorHelper.cpp)

    target_link_libraries(TensorflowDetector YARP::YARP_OS
                                             YARP::YARP_init
                                             YARP::YARP_dev
                                             YARP::YARP_sig
                                             ROBOTICSLAB::ColorDebug
                                             TensorflowCC::Shared
                                             ${OpenCV_LIBS})

    # switchDetection2D
    add_executable(switchDetection2D main.cpp
                                     SwitchDetection2D.hpp
                                     SwitchDetection2D.cpp
                                     DetectorThread.hpp
                                     DetectorThread.cpp
                                     Detector.hpp)

    target_link_libraries(switchDetection2D YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_dev
                                            YARP::YARP_sig
                                            ROBOTICSLAB::ColorDebug
                                            ColorRegionDetector
                                            HaarDetector
                                            TensorflowDetector)

    install(TARGETS switchDetection2D
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_switchDetection2D OFF CACHE BOOL "Enable/disable switchDetection2D program" FORCE)

endif()
