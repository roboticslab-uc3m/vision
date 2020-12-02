// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_programs
 *
 * @defgroup rgbDetection rgbDetection
 *
 * @brief Creates an instance of roboticslab::RgbDetection.
 *
 * @section rgbDetectionOptions rgbDetection options:
 *
 * | PROPERTY     | DESCRIPTION                          | DEFAULT               |
 * |--------------|--------------------------------------|-----------------------|
 * | help         |                                      |                       |
 * | from         | file.ini                             | rgbDetection.ini |
 * | context      | context name                         | rgbDetection     |
 * | cropSelector |                                      | 0                     |
 * | cameraDevice | device we create                     | remote_grabber        |
 * | cameraLocal  | if accesing remote, local port name  | /rgbDetection    |
 * | cameraRemote | if accesing remote, remote port name | /grabber              |
 * | watchdog     |                                      | 2.000000              |
 *
 *
 * @section  rgbDetectionPorts Detector output ports:
 *
 * | OUTPUT PORT                | CONTENT                                                 |
 * |----------------------------|---------------------------------------------------------|
 * | /rgbDetection/img:o   | output camera image with object detection using squares |
 * | /rgbDetection/state:o | detected objects                                        |
 *
 * @section detectorThread Detector thread options:
 *
 * | PROPERTY   | DESCRIPTION     | DEFAULT      |
 * |------------|-----------------|--------------|
 * | help       |                 |              |
 * | from       | file.ini        |              |
 * | context    | context name    |              |
 * | rateMs     |                 | 20           |
 * | detector   | detector device | HaarDetector |
 */

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <ColorDebug.h>

#include "RgbDetection.hpp"

#define DEFAULT_CONTEXT    "rgbDetection"
#define DEFAULT_CONFIG_FILE    "rgbDetection.ini"

int main(int argc, char** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);;
    rf.setDefaultContext(DEFAULT_CONTEXT);
    rf.setDefaultConfigFile(DEFAULT_CONFIG_FILE);
    rf.configure(argc, argv);

    roboticslab::RgbDetection mod;

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
