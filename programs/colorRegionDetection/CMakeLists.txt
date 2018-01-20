cmake_dependent_option(ENABLE_colorRegionDetection "Enable/disable colorRegionDetection program" ON
                       ENABLE_TravisLib OFF)

if(ENABLE_colorRegionDetection)

find_package(OpenCV REQUIRED)
find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_executable(colorRegionDetection main.cpp
                                    ColorRegionDetection.hpp
                                    ColorRegionDetection.cpp
                                    SegmentorThread.hpp
                                    SegmentorThread.cpp)

target_link_libraries(colorRegionDetection YARP::YARP_OS
                                           YARP::YARP_init
                                           YARP::YARP_dev
                                           YARP::YARP_sig
                                           ${OpenCV_LIBS}
                                           TravisLib)

install(TARGETS colorRegionDetection
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
