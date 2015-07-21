// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DM1_HPP__
#define __DM1_HPP__

#include <yarp/os/all.h>
#include <stdlib.h>

#include "StateMachine.hpp"

//j//#define DEFAULT_FILE_NAME "segRec_ecf_params.xml"

using namespace yarp::os;

class Dm1 : public RFModule {
  private:
    StateMachine _stateMachine;
    BufferedPort<Bottle> _inAsrPort;
    BufferedPort<Bottle> _inFeaturesPort;
    Port _outPointsPort;
    Port _outTextPort;
    Port _outTtsPort;
    RpcClient _fittingClient;
    RpcClient _groundingClient;
    RpcClient _solverClient;

    bool interruptModule();
    double getPeriod();
    bool updateModule();

  public:
    bool configure(ResourceFinder &rf);
};

#endif  // __DM1_HPP__
