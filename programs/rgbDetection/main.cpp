// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_programs
 * @defgroup rgbDetection rgbDetection
 * @brief Creates an instance of roboticslab::RgbDetection.
 *
 * @section rgbDetectionOptions Options
 *
 * | PROPERTY     | DESCRIPTION                          | DEFAULT          |
 * |--------------|--------------------------------------|------------------|
 * | from         | file.ini                             | rgbDetection.ini |
 * | context      | context name                         | rgbDetection     |
 * | sensorDevice | sensor device name                   | remote_grabber   |
 * | sensorRemote | if accesing remote, remote port name | /grabber         |
 * | localPrefx   | local port name prefix               | /rgbDetection    |
 * | period       | update period (seconds)              | 0.02             |
 * | detector     | detector device name                 |                  |
 *
 * @section rgbDetectionInputPorts Input ports
 *
 * | PORT                 | CONTENT                                                               |
 * |----------------------|-----------------------------------------------------------------------|
 * | <localPrefix>/crop:i | 4-int bottle with (x1,y1,x2,y2) vertices of the rectangular crop area |
 *
 * @section rgbDetectionOutputPorts Output ports
 *
 * | PORT                  | CONTENT                                                 |
 * |-----------------------|---------------------------------------------------------|
 * | <localPrefix>/img:o   | output camera image with object detection using squares |
 * | <localPrefix>/state:o | detected objects                                        |
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "RgbDetection.hpp"

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("rgbDetection");
    rf.setDefaultConfigFile("rgbDetection.ini");
    rf.configure(argc, argv);

    roboticslab::RgbDetection mod;

    if (rf.check("help"))
    {
        mod.configure(rf);
        return 1;
    }

    yInfo() << "Add --help for usage notes";
    yInfo() << "Checking for yarp network...";

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "No YARP network found (try running \"yarpserver &\")";
        return 1;
    }

    return mod.runModule(rf);
}
