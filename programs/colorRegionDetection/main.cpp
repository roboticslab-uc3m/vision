// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup vision_programs
 *
 * \defgroup colorRegionDetection colorRegionDetection
 *
 * @brief Creates an instance of roboticslab::ColorRegionDetection.
 *
 * @section colorRegionDetection_options ColorRegionDetection options:
 *
 * |PROPERTY     | DESCRIPTION                           | DEFAULT               |
 * |-------------|---------------------------------------|-----------------------|
 * |help         |                                       |                       |
 * |from         |file.ini                               |                       |
 * |context      |path                                   |                       |
 * |cropSelector |                                       | 0                     |
 * |RGBDDevice   |device we create                       | RGBDSensorClient      |
 * |RGBDLocal    |if accesing remote, local port name    | /colorRegionDetection |
 * |RGBDRemote   |if accesing remote, remote port name   | /rgbd                 |
 * |watchdog     |                                       | 2.000000              |
 *
 *
 * @section segmentorthread_options SegmentorThread options:
 *
 * |PROPERTY             | DESCRIPTION                   | DEFAULT              |
 * |---------------------|-------------------------------|----------------------|
 * |help                 |                               |                      |
 * |from                 |file.ini                       |                      |
 * |context              |path                           |                      |
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

#include "ColorRegionDetection.hpp"

int main(int argc, char** argv) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("colorRegionDetection");
    rf.setDefaultConfigFile("colorRegionDetection.ini");
    rf.configure(argc, argv);

    roboticslab::ColorRegionDetection mod;
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

