// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_programs
 *
 * @defgroup haarDetection2D haarDetection2D
 *
 * @brief Creates an instance of roboticslab::HaarDetection2D.
 *
 * @section haarDetection2DOptions HaarDetection2D options:
 *
 * | PROPERTY     | DESCRIPTION                          | DEFAULT          |
 * |--------------|--------------------------------------|------------------|
 * | help         |                                      |                  |
 * | from         | file.ini                             |                  |
 * | context      | path                                 |                  |
 * | cropSelector |                                      | 0                |
 * | cameraDevice | device we create                     | remote_grabber   |
 * | cameraLocal  | if accesing remote, local port name  | /haarDetection2D |
 * | cameraRemote | if accesing remote, remote port name | /frameGrabber2D  |
 * | watchdog     |                                      | 2.000000         |
 *
 *
 * @section  haarDetection2DPorts HaarDetection2D output ports:
 *
 * | OUTPUT PORT              | CONTENT                                                 |
 * |--------------------------|---------------------------------------------------------|
 * | /haarDetection2D/img:o   | Output camera image with object detection using squares |
 * | /haarDetection2D/state:o | xy coordinates of object detection                      |
 *
 * @section segmentorThread SegmentorThread options:
 *
 * | PROPERTY   | DESCRIPTION | DEFAULT                      |
 * |------------|-------------|------------------------------|
 * | help       |             |                              |
 * | from       | file.ini    |                              |
 * | context    | path        |                              |
 * | fx_d       |             | 525.000000                   |
 * | fy_d       |             | 525.000000                   |
 * | cx_d       |             | 319.500000                   |
 * | cy_d       |             | 239.500000                   |
 * | rateMs     |             | 20                           |
 * | xmlCascade | file.xml    | haarcascade_cocacola_can.xml |
 */

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "HaarDetection2D.hpp"
#include "ColorDebug.hpp"

int main(int argc, char** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("haarDetection");
    rf.setDefaultConfigFile("haarDetection2D.ini");
    rf.configure(argc, argv);

    roboticslab::HaarDetection2D mod;

    if (rf.check("help"))
    {
        return mod.runModule(rf);
    }

    CD_INFO("Run \"%s --help\" for options.\n", argv[0]);
    CD_INFO("%s checking for yarp network... ", argv[0]);

    std::fflush(stdout);

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("%s found no yarp network (try running \"yarpserver &\"), bye!\n", argv[0]);
        return 1;
    }
    else
    {
        CD_SUCCESS_NO_HEADER("[ok]\n");
    }

    return mod.runModule(rf);
}
