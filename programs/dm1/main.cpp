// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * 
 * @ingroup xgnitive_modules
 * \defgroup dm1 dm1
 *
 * @brief The dm1 module provides basic 3D imagination.
 *
 * The dm1 module provides basic 3D imagination.
 * 
 * @section dm1_legal Legal
 *
 * Copyright: 2013 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=72">Juan G. Victores</a>,
 * <a href="http://www.mendeley.com/profiles/santiago-morante-cendrero/">Santiago Morante</a>
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 *
 * <hr>
 *
 * This file can be edited at $XGNITIVE_ROOT/main/src/modules/dm1
 *
 */

#include <yarp/os/all.h>

#include "Dm1.hpp"

using namespace yarp::os;

int main(int argc, char **argv) {

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("dm1/conf");
    rf.setDefaultConfigFile("dm1.ini");
    rf.configure("XGNITIVE_ROOT", argc, argv);

    Dm1 mod;
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

