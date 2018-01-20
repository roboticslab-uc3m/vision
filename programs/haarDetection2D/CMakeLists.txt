option(ENABLE_haarDetection2D "Enable/disable haarDetection2D program" ON)

if(ENABLE_haarDetection2D)

find_package(OpenCV REQUIRED)
find_package(YARP REQUIRED)

get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_VISION_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${_common_includes})

add_executable(haarDetection2D main.cpp
                               HaarDetection2D.hpp
                               HaarDetection2D.cpp
                               SegmentorThread.hpp
                               SegmentorThread.cpp)

add_dependencies(haarDetection2D COLOR_DEBUG)

target_link_libraries(haarDetection2D YARP::YARP_OS
                                      YARP::YARP_init
                                      YARP::YARP_dev
                                      YARP::YARP_sig
                                      ${OpenCV_LIBS})

install(TARGETS haarDetection2D
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
