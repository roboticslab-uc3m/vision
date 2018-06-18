if(NOT YARP_VERSION_SHORT VERSION_LESS 3 AND (NOT DEFINED ENABLE_voxelOccupancyDetection OR ENABLE_voxelOccupancyDetection))
    message(WARNING "Unsuitable YARP version (>=3), disabling voxelOccupancyDetection program")
endif()

cmake_dependent_option(ENABLE_voxelOccupancyDetection "Enable/disable voxelOccupancyDetection program" ON
                       "YARP_VERSION_SHORT VERSION_LESS 3" OFF)

if(ENABLE_voxelOccupancyDetection)

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

else()

    set(ENABLE_voxelOccupancyDetection OFF CACHE BOOL "Enable/disable voxelOccupancyDetection program" FORCE)

endif()
