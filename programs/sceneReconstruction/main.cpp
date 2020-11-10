// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_programs
 *
 * @defgroup sceneReconstruction sceneReconstruction
 *
 * @brief Creates an instance of roboticslab::SceneReconstruction.
 */

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <ColorDebug.h>

#include "SceneReconstruction.hpp"

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("sceneReconstruction");
    rf.setDefaultConfigFile("sceneReconstruction");
    rf.configure(argc, argv);

    roboticslab::SceneReconstruction mod;

    CD_INFO("%s checking for yarp network... ", argv[0]);

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
