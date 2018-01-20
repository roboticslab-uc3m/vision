option(ENABLE_haarDetection "Enable/disable haarDetection program" ON)

if(ENABLE_haarDetection)

find_package(OpenCV REQUIRED)
find_package(YARP REQUIRED)

get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_VISION_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${_common_includes})

add_executable(haarDetection main.cpp
                             HaarDetection.hpp
                             HaarDetection.cpp
                             SegmentorThread.hpp
                             SegmentorThread.cpp)

target_link_libraries(haarDetection YARP::YARP_OS
                                    YARP::YARP_init
                                    YARP::YARP_dev
                                    YARP::YARP_sig
                                    ${OpenCV_LIBS})

install(TARGETS haarDetection
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
