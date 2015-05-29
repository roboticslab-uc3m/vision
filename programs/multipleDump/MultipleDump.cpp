// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MultipleDump.hpp"

/************************************************************************/
bool MultipleDump::configure(ResourceFinder &rf) {

    watchdog = DEFAULT_WATCHDOG;  // double

    //-- File for writing: naming and opening.
    std::string filePrefix("dump");
    if(rf.check("file"))
        filePrefix = rf.find("file").asString();
    // [thanks] http://www.tutorialspoint.com/cplusplus/cpp_date_time.htm
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::stringstream fileName;
    fileName << filePrefix;
    fileName << "_";
    fileName << (ltm->tm_mday);
    fileName << ".";
    //fileName << 1 + ltm->tm_mon;
    char buffer [4];  // 'M' 'a' 'y' '\0'
    strftime (buffer,4,"%b",ltm);
    fileName << buffer;
    fileName << ".";
    fileName << 1900 + ltm->tm_year;
    fileName << "_";
    fileName << ltm->tm_hour;  // not +1 as in tutorial
    fileName << ":";
    fileName << ltm->tm_min;  // not +1 as in tutorial
    fileName << ":";
    fileName << ltm->tm_sec;  // not +1 as in tutorial
    fileName << ".csv";
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

