// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MULTIPLE_DUMP_HPP__
#define __MULTIPLE_DUMP_HPP__

#include <sstream>

#include "MultipleDumpThread.hpp"

#define DEFAULT_WATCHDOG    2       // [s]

using namespace yarp::os;

class MultipleDump : public RFModule {

    public:
        bool configure(ResourceFinder &rf);

    protected:
        MultipleDumpThread multipleDumpThread;

        bool interruptModule();
        double getPeriod();
        bool updateModule();
        double watchdog;

        BufferedPort<Bottle> in1;
        BufferedPort<Bottle> in2;

        FILE * filePtr;
};

#endif  // __MULTIPLE_DUMP_HPP__

