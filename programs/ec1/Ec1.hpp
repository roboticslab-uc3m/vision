// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EC1_HPP__
#define __EC1_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

using namespace yarp::os;

class InCvPort : public BufferedPort<Bottle> {
    public:
        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->iPositionControl = iPositionControl;
        }

    protected:
        virtual void onRead(Bottle& b) {
            if (b.size() < 3) return;
            double x = b.get(0).asDouble();
            double y = b.get(1).asDouble();
            double z = b.get(2).asDouble();
            printf("%f %f %f\n",x,y,z);
            if( x > 50 ) iPositionControl->relativeMove(0, -5);
            if( x < -50 ) iPositionControl->relativeMove(0, 5);
            //iPositionControl->positionMove(0,0.0);
            //iPositionControl->positionMove(1,0.0);
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
            switch ( b.get(0).asVocab() ) {
                case VOCAB_FOLLOW_ME:
                    printf("enabling callback\n");
                    inCvPortPtr->useCallback();
                    break;
                case VOCAB_STOP_FOLLOWING:
                    printf("disabling callback\n");
                    inCvPortPtr->disableCallback();
                    break;
                default:
                    break;
            }
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
