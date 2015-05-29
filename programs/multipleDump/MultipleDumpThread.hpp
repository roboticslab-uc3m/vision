// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MULTIPLE_DUMP_THREAD_HPP__
#define __MULTIPLE_DUMP_THREAD_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>

#define DEFAULT_RATE_MS 20

using namespace yarp::os;

class MultipleDumpThread : public RateThread {

    virtual void run();

public:
    MultipleDumpThread() : RateThread(DEFAULT_RATE_MS) {}

};

#endif  // __MULTIPLE_DUMP_THREAD_HPP__

