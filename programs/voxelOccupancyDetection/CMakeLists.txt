# disabled, see https://github.com/roboticslab-uc3m/vision/issues/106
option(ENABLE_voxelOccupancyDetection "Enable/disable voxelOccupancyDetection program" OFF)

if(ENABLE_voxelOccupancyDetection)

    add_executable(voxelOccupancyDetection main.cpp
                                           VoxelOccupancyDetection.hpp
                                           VoxelOccupancyDetection.cpp
                                           SegmentorThread.hpp
                                           SegmentorThread.cpp)

    target_link_libraries(voxelOccupancyDetection YARP::YARP_os
                                                  YARP::YARP_init
                                                  YARP::YARP_dev
                                                  YARP::YARP_sig)

    install(TARGETS voxelOccupancyDetection)

endif()
