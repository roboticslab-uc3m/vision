// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchHead.hpp"

using namespace yarp::os;
using namespace yarp::dev;

YARP_DECLARE_PLUGINS(HeadYarp)

int main(int argc, char *argv[]) {

    YARP_REGISTER_PLUGINS(HeadYarp);

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("launchHead");
    rf.setDefaultConfigFile("launchHead.ini");
    rf.configure(argc, argv);

    CD_INFO("Checking for yarp network...\n");
    Network yarp;
    if (!yarp.checkNetwork()) {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return -1;
    }
    CD_SUCCESS("Found yarp network.\n");

    teo::LaunchHead mod;
    return mod.runModule(rf);
}

