// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_head_programs
 *
 * \defgroup cvFaces cvFaces
 *
 * @brief Creates an instance of teo::CvFaces.
 *
 * @section cvfaces_options CvFaces options:
 *
 * |PROPERTY            | DESCRIPTION                           | DEFAULT              |
 * |--------------------|---------------------------------------|----------------------|
 * |help                |                                       |                      |
 * |from                |file.ini                               |                      |
 * |context             |path                                   |                      |
 * |cropSelector        |                                       |0                     |
 * |kinectDevice        |device we create                       |OpenNI2DeviceServer   |
 * |kinectLocal         |if accesing remote, local port name    |/cvFaces              |
 * |kinectRemote        |if accesing remote, remote port name   |/OpenNI2              |
 * |watchdog            |                                       |2.000000              |
 *
 * @section cvfaces_options SegmentorThread options:
 *
 * |PROPERTY            | DESCRIPTION                           | DEFAULT              |
 * |--------------------|---------------------------------------|----------------------|
 * |help                |                                       |                      |
 * |from                |file.ini                               |                      |
 * |context             |path                                   |                      |
 * |fx_d                |                                       |525.000000            |
 * |fy_d                |                                       |525.000000            |
 * |cx_d                |                                       |319.500000            |
 * |cy_d                |                                       |239.500000            |
 * |rateMs              |                                       |20                    |
 *
 *
 *


*/

#include "CvFaces.hpp"

int main(int argc, char** argv) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cvFaces");
    rf.setDefaultConfigFile("cvFaces.ini");
    rf.configure(argc, argv);

    teo::CvFaces mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"%s --help\" for options.\n",argv[0]);
    printf("%s checking for yarp network... ",argv[0]);
    fflush(stdout);
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stderr,"[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n",argv[0]);
        return -1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}

