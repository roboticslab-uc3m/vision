if(ENABLE_haarDetection2D)

find_package(OpenCV REQUIRED)
find_package(YARP REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS})

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
        DESTINATION bin)

endif()
