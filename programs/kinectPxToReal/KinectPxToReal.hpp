// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
  * Copyright 2012 UC3M
  * Author: Juan G. Victores
  */

#ifndef __KINECT_PX_TO_REAL_HPP__
#define __KINECT_PX_TO_REAL_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/all.h>

#include "CallbackPort.hpp"

#define DEFAULT_FX          640     //
#define DEFAULT_FY          640     //
#define DEFAULT_CX          320     //
#define DEFAULT_CY          240     //
#define DEFAULT_WATCHDOG    5       // [s]

using namespace yarp::os;
using namespace yarp::sig;

class KinectPxToReal : public RFModule {
    protected:
        bool updateModule();
        bool interruptModule();
        double getPeriod();
        double watchdog; // [s]

        BufferedPort<ImageOf<PixelFloat> > depthPort;
        CallbackPort callbackPort;
        Port outPort;

    public:
        bool configure(ResourceFinder &rf);
};

#endif  // __KINECT_PX_TO_REAL_HPP__

