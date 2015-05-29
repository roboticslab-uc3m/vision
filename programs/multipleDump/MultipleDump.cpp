// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MultipleDump.hpp"

/************************************************************************/
bool MultipleDump::configure(ResourceFinder &rf) {

    watchdog = DEFAULT_WATCHDOG;  // double

    multipleDumpThread.setIn1(&in1);
    multipleDumpThread.setIn2(&in2);
    in1.open("/in1");
    in2.open("/in2");

    multipleDumpThread.start();
    return true;
}

/*****************************************************************/
double MultipleDump::getPeriod() {
    return watchdog;  // [s]
}

/************************************************************************/

bool MultipleDump::updateModule() {
    printf("MultipleDump alive...\n");
    return true;
}

/************************************************************************/

bool MultipleDump::interruptModule() {
    printf("MultipleDump closing...\n");
    return true;
}

/************************************************************************/

