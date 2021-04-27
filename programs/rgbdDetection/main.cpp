// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_programs
 * @defgroup rgbdDetection rgbdDetection
 * @brief Creates an instance of roboticslab::RgbdDetection.
 *
 * @section rgbdDetectionOptions Options
 *
 * | PROPERTY     | DESCRIPTION                          | DEFAULT           |
 * |--------------|--------------------------------------|-------------------|
 * | from         | file.ini                             | rgbdDetection.ini |
 * | context      | context name                         | rgbdDetection     |
 * | sensorDevice | sensor device name                   | RGBDSensorClient  |
 * | sensorRemote | if accesing remote, remote port name | /rgbd             |
 * | localPrefx   | local port name prefix               | /rgbdDetection    |
 * | period       | update period (seconds)              | 0.02              |
 * | detector     | detector device name                 |                   |
 *
 * @section rgbdDetectionPorts Output ports
 *
 * | PORT                  | CONTENT                                                 |
 * |-----------------------|---------------------------------------------------------|
 * | <localPrefix>/img:o   | output camera image with object detection using squares |
 * | <localPrefix>/state:o | xyz coordinates of object detection (meters)            |
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "RgbdDetection.hpp"

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("rgbdDetection");
    rf.setDefaultConfigFile("rgbdDetection.ini");
    rf.configure(argc, argv);

    roboticslab::RgbdDetection mod;

    if (rf.check("help"))
    {
        mod.configure(rf);
        return 1;
    }

    yInfo() << "Run \"%s --help\" for options" << argv[0];
    yInfo() << argv[0] << "Checking for yarp network...";

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << argv[0] << "found no yarp network (try running \"yarpserver &\"), bye!";
        return 1;
    }

    return mod.runModule(rf);
}
