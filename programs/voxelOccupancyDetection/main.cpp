// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup vision_programs
 *
 * \defgroup voxelOccupancyDetection voxelOccupancyDetection
 *
 * @brief Creates an instance ---- of roboticslab::VoxelOccupancyDetection.
 *
 * @section voxelOccupancyDetection_options VoxelOccupancyDetection options:
 *
 * |PROPERTY     | DESCRIPTION                           | DEFAULT              |
 * |-------------|---------------------------------------|----------------------|
 * |help         |                                       |                      |
 * |from         |file.ini                               |                      |
 * |context      |path                                   |                      |
 * |cropSelector |                                       | 0                    |
 * |kinectDevice |device we create                       | OpenNI2DeviceServer  |
 * |kinectLocal  |if accesing remote, local port name    | /voxelOccupancyDetection                 |
 * |kinectRemote |if accesing remote, remote port name   | /OpenNI2             |
 * |watchdog     |                                       | 2.000000             |
 *
 *
 * @section segmentorthread_options SegmentorThread options:
 *
 * |PROPERTY             | DESCRIPTION                   | DEFAULT              |
 * |---------------------|-------------------------------|----------------------|
 * |help                 |                               |                      |
 * |from                 |file.ini                       |                      |
 * |context              |path                           |                      |
 * |fx_d                 |                               |525.000000            |
 * |fy_d                 |                               |525.000000            |
 * |cx_d                 |                               |319.500000            |
 * |cy_d                 |                               |239.500000            |
 * |fx_rgb               |                               |525.000000            |
 * |fy_rgb               |                               |525.000000            |
 * |cx_rgb               |                               |319.500000            |
 * |cy_rgb               |                               |239.500000            |
 * |algorithm            |                               |blueMinusRed          |
 * |locate               |centroid or bottom             |centroid              |
 * |maxNumBlobs          |                               |2                     |
 * |morphClosing         |percentage, 2 or 4 okay        |2.000000              |
 * |morphOpening         |percentage, 2 or 4 okay        |0.000000              |
 * |outFeatures          |mmX, mmY, mmZ, pxXpos, pxYpos, pxX, pxY, angle, area, aspectRatio, rectangularity, axisFirst, axisSecond solidity, hue, sat, val, hueStdDev, satStdDev, valStdDev, time |(mmX mmY mmZ)
 * |outFeaturesFormat    |0=bottled,1=minimal            |0                     |
 * |outImage             |0=rgb,1=bin                    |1                     |
 * |rateMs               |                               |20                    |
 * |seeBounding          |0=none,1=box,2=contour,3=both  |3                     |
 * |threshold            |                               |55                    |
 *
 *
 */

#include "VoxelOccupancyDetection.hpp"

int main(int argc, char** argv) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("voxelOccupancyDetection");
    rf.setDefaultConfigFile("voxelOccupancyDetection.ini");
    rf.configure(argc, argv);

    roboticslab::VoxelOccupancyDetection mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"%s --help\" for options.\n",argv[0]);
    printf("%s checking for yarp network... ",argv[0]);
    fflush(stdout);
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stderr,"[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n",argv[0]);
        return 1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}

