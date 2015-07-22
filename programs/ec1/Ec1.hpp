// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EC1_HPP__
#define __EC1_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>


using namespace yarp::os;

class InSrPort : public BufferedPort<Bottle> {
     virtual void onRead(Bottle& b) {
          // process data in b
     }
};

class InCvPort : public BufferedPort<Bottle> {
     virtual void onRead(Bottle& b) {
          // process data in b
     }
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
