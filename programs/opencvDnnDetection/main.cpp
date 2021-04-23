// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_programs
 *
 * @defgroup opencvDnnDetection opencvDnnDetection
 *
 * @brief Creates an instance of roboticslab::OpencvDnnDetection.
 */

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <ColorDebug.h>

#include "OpencvDnnDetection.hpp"

#define DEFAULT_CONTEXT    "opencvDnnDetection"
#define DEFAULT_CONFIG_FILE    "opencvDnnDetection.ini"

int main(int argc, char** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);;
    rf.setDefaultContext(DEFAULT_CONTEXT);
    rf.setDefaultConfigFile(DEFAULT_CONFIG_FILE);
    rf.configure(argc, argv);

    roboticslab::OpencvDnnDetection mod;

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
