// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
  * Copyright 2012 UC3M
  * Author: Juan G. Victores
  */

#ifndef __KINECT_PX_TO_REAL_HPP__
#define __KINECT_PX_TO_REAL_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Port.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>

#include "CallbackPort.hpp"

#define DEFAULT_RGBD_DEVICE "RGBDSensorClient"
#define DEFAULT_RGBD_LOCAL "/kinectPxToReal"
#define DEFAULT_RGBD_REMOTE "/rgbd"
#define DEFAULT_WATCHDOG    5       // [s]

class KinectPxToReal : public yarp::os::RFModule {
    protected:
        bool updateModule();
        bool interruptModule();
        double getPeriod();
        double watchdog; // [s]

        CallbackPort callbackPort;
        yarp::os::Port outPort;

        yarp::dev::PolyDriver rgbdDevice;
        yarp::dev::IRGBDSensor * irgbdSensor;

    public:
        bool configure(yarp::os::ResourceFinder &rf);
};

#endif  // __KINECT_PX_TO_REAL_HPP__
