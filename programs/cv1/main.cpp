// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_head_programs
 *
 * \defgroup cv1 cv1
 *
 * @brief Creates an instance teo::Cv1.
 */

#include "Cv1.hpp"

int main(int argc, char** argv) {

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cv1");
    rf.setDefaultConfigFile("cv1.ini");
    rf.configure(argc, argv);

    teo::Cv1 mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"%s --help\" for options.\n",argv[0]);
    printf("%s checking for yarp network... ",argv[0]);
    fflush(stdout);
    Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stderr,"[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n",argv[0]);
        return -1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}

