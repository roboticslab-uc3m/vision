// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EC1_HPP__
#define __EC1_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>


using namespace yarp::os;

class InCvPort : public BufferedPort<Bottle> {
    public:
        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->iPositionControl = iPositionControl;
        }

    protected:
        virtual void onRead(Bottle& b) {
            double x = b.get(0).asDouble();
            double y = b.get(1).asDouble();
            double z = b.get(2).asDouble();
            iPositionControl->positionMove(0,0.0);
            iPositionControl->positionMove(1,0.0);
        }
        yarp::dev::IPositionControl *iPositionControl;
};

class InSrPort : public BufferedPort<Bottle> {
    public:
        void setInCvPortPtr(BufferedPort<Bottle> *inCvPortPtr) {
            this->inCvPortPtr = inCvPortPtr;
        }

    protected:
        virtual void onRead(Bottle& b) {
            // process data in b
        }
        BufferedPort<Bottle>* inCvPortPtr;
};

class Ec1 : public RFModule {
private:
    InSrPort inSrPort;
    InCvPort inCvPort;
    yarp::dev::PolyDriver headDevice;
    yarp::dev::IPositionControl *iPositionControl;

    bool interruptModule();
    double getPeriod();
    bool updateModule();

  public:
    bool configure(ResourceFinder &rf);
};

#endif  // __EC1_HPP__
