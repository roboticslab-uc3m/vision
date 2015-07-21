// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StateMachine.hpp"

/************************************************************************/

bool StateMachine::threadInit() {
    _machineState = 0;
    return true;
}

/************************************************************************/

void StateMachine::run() {
    while(!isStopping()) {
        if(_machineState==-1) {
            ttsSay( ConstString("Sorry, I do not know what that is.") );
            _machineState=0;
        } else if(_machineState==0) {
            ttsSay( ConstString("I am ready. Please tell me.") );
            _machineState=1;
        } else if(_machineState==1) {
            ConstString inStr = asrListen();
            // Blocking
            _inStrState1 = inStr;
            if( _inStrState1.find("follow me") != ConstString::npos ) _machineState=2;
            else if ( _inStrState1.find("stop following") != ConstString::npos ) _machineState=3;
            else _machineState=-1;
        } else if (_machineState==2) {
            ttsSay( ConstString("Okay, I will follow you.") );
            _machineState=0;
        } else if (_machineState==3) {
            ttsSay( ConstString("Okay, I will stop following you") );
        } else {
            _machineState=0;
            ttsSay( ConstString("ANOMALY") );
        }         
    }
}

/************************************************************************/

void StateMachine::ttsSay(const ConstString& sayConstString) {
    Bottle bOut;
    bOut.addString(sayConstString);
    outTtsPort->write(bOut);
    printf("[StateMachine] Said: %s\n", sayConstString.c_str());
}

/************************************************************************/

ConstString StateMachine::asrListen() {
    Bottle* bIn = inSrPort->read(true);  // shouldWait
    printf("[StateMachine] Listened: %s\n", bIn->toString().c_str());
    return bIn->get(0).asString();
}

/************************************************************************/

int StateMachine::getMachineState() {
    return _machineState;
}

/************************************************************************/

void StateMachine::setInSrPort(yarp::os::BufferedPort<yarp::os::Bottle>* inSrPort) {
    this->inSrPort = inSrPort;
}

/************************************************************************/

void StateMachine::setInCvPort(yarp::os::BufferedPort<yarp::os::Bottle>* inCvPort) {
    this->inCvPort = inCvPort;
}

/************************************************************************/

void StateMachine::setOutCmdPort(yarp::os::Port* outCmdPort) {
    this->outCmdPort = outCmdPort;
}

/************************************************************************/

void StateMachine::setOutTtsPort(yarp::os::Port* outTtsPort) {
    this->outTtsPort = outTtsPort;
}

/************************************************************************/
