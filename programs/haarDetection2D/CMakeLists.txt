if(ENABLE_haarDetection2D)

find_package(OpenCV REQUIRED)
find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(haarDetection2D main.cpp
                               HaarDetection2D.hpp
                               HaarDetection2D.cpp
                               SegmentorThread.hpp
                               SegmentorThread.cpp)

target_link_libraries(haarDetection2D ${OpenCV_LIBS} ${YARP_LIBRARIES})

install(TARGETS haarDetection2D
        DESTINATION bin)

endif()
