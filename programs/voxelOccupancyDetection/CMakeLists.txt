option(ENABLE_voxelOccupancyDetection "Enable/disable voxelOccupancyDetection program" ON)

if(ENABLE_voxelOccupancyDetection)

find_package(YARP REQUIRED)

get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_VISION_INCLUDE_DIRS)

include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_includes})

add_executable(voxelOccupancyDetection main.cpp
                                       VoxelOccupancyDetection.hpp
                                       VoxelOccupancyDetection.cpp
                                       SegmentorThread.hpp
                                       SegmentorThread.cpp)

target_link_libraries(voxelOccupancyDetection YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig)

install(TARGETS voxelOccupancyDetection
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
