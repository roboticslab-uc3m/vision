option(ENABLE_voxelOccupancyDetection "Enable/disable voxelOccupancyDetection program" ON)

if(ENABLE_voxelOccupancyDetection)

if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
    include_directories(${YARP_INCLUDE_DIRS})
endif()

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
