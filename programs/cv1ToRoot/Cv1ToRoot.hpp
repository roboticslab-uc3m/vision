// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
  * Copyright 2012 UC3M
  * This file is part of Robot Devastation Game: ECRO Version
  * Author: Juan G. Victores
  */

#ifndef __PREMULT_H_HPP__
#define __PREMULT_H_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Matrix.h>

#include "PremultPorts.hpp"

#define DEFAULT_WATCHDOG    5       // [s]

using namespace yarp::os;

class Cv1ToRoot : public RFModule {
    protected:
        bool updateModule();
        bool interruptModule();
        double getPeriod();
        double watchdog; // [s]

        Port outPort;
        PremultPorts premultPorts;

    public:
        bool configure(ResourceFinder &rf);
};

#endif  // __PREMULT_H_HPP__

