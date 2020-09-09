// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CALLBACK_PORT_HPP__
#define __CALLBACK_PORT_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>

#include <yarp/dev/IRGBDSensor.h>

class CallbackPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
    private:
        yarp::os::Port* outPort;
        virtual void onRead(yarp::os::Bottle& b);
        double fx, fy, cx, cy;
        yarp::dev::IRGBDSensor * irgbdSensor;

    public:
        void setParams(double _fx, double _fy, double _cx, double _cy);
        void setDepthSensorHandle(yarp::dev::IRGBDSensor * irgbdSensor);
        void setOutPort(yarp::os::Port* _outPort);
};

#endif  // __CALLBACK_PORT_HPP__
