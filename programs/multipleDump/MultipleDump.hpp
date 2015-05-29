// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CV1_HPP__
#define __CV1_HPP__

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

  public:
    bool configure(ResourceFinder &rf);
};

#endif  // __CV1_HPP__

