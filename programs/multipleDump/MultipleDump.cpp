// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MultipleDump.hpp"

namespace roboticslab
{

/************************************************************************/
bool MultipleDump::configure(yarp::os::ResourceFinder &rf) {

    watchdog = DEFAULT_WATCHDOG;  // double

    //-- File for writing: naming and opening.
    std::string filePrefix("dump");
    if(rf.check("file"))
        filePrefix = rf.find("file").asString();
    // [thanks] http://www.cplusplus.com/reference/ctime/strftime/
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::stringstream fileName;
    fileName << filePrefix;
    char buffer [80];  // At least: 20.Mar.2015_03:26:26.csv
    strftime (buffer,80,"_%d.%b.%Y_%H:%M:%S.csv",ltm);
    fileName << buffer;
    filePtr = ::fopen (fileName.str().c_str(),"w");
    if( ! filePtr ) {
        printf("Could not open file: %s\n",fileName.str().c_str());
        return false;
    }
    printf("Opened file: %s\n",fileName.str().c_str());

    //-- Configure the thread.
    multipleDumpThread.setFilePtr(filePtr);
    multipleDumpThread.setIn1(&in1);
    multipleDumpThread.setIn2(&in2);

    //-- Open the ports.
    in1.open("/in1");
    in2.open("/in2");

    //-- Start the thread.
    return multipleDumpThread.start();
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

}  // namespace roboticslab
