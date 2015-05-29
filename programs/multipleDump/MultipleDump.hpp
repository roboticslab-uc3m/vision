// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MULTIPLE_DUMP_HPP__
#define __MULTIPLE_DUMP_HPP__

#include "MultipleDumpThread.hpp"

#define DEFAULT_WATCHDOG    2       // [s]

using namespace yarp::os;

class MultipleDump : public RFModule {
  private:
    MultipleDumpThread multipleDumpThread;

    bool interruptModule();
    double getPeriod();
    bool updateModule();
    double watchdog;

    BufferedPort<Bottle> in1;
    BufferedPort<Bottle> in2;

  public:
    bool configure(ResourceFinder &rf);
};

#endif  // __MULTIPLE_DUMP_HPP__

