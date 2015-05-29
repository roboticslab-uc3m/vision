// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MultipleDump.hpp"

/************************************************************************/
bool MultipleDump::configure(ResourceFinder &rf) {

    watchdog = DEFAULT_WATCHDOG;  // double

    //-- Open file for writing.
    std::string fileName = rf.check("file",yarp::os::Value(DEFAULT_FILE_NAME),"file name").asString();
    printf("Using file: %s (default: " DEFAULT_FILE_NAME ").\n",fileName.c_str());
    filePtr = ::fopen (fileName.c_str(),"w");
    if( ! filePtr ) {
        printf("Could not open file: %s.\n",fileName.c_str());
        return false;
    }
    printf("Opened file: %s.\n",fileName.c_str());

    //-- Configure the thread.
    multipleDumpThread.setFilePtr(filePtr);
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

    multipleDumpThread.stop();
    ::fclose(filePtr);

    return true;
}

/************************************************************************/

