// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Dm1.hpp"

/************************************************************************/

bool Dm1::configure(ResourceFinder &rf) {

    //ConstString fileName(DEFAULT_FILE_NAME);
    
    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Dm1 options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        //printf("\t--file (default: \"%s\")\n",fileName.c_str());
    }
    //if (rf.check("file")) fileName = rf.find("file").asString();
    //printf("Dm1 using file: %s\n",fileName.c_str());

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //-----------------OPEN LOCAL PORTS------------//
    _outPointsPort.open("/dm1/points:o");
    _outTextPort.open("/dm1/text:o");
    _outTtsPort.open("/dm1/tts:o");
    _inAsrPort.open("/dm1/asr:i");
    _inFeaturesPort.open("/dm1/features:i");
    _fittingClient.open("/dm1/fitting/rpc:o");
    _groundingClient.open("/dm1/grounding/rpc:o");
    _solverClient.open("/dm1/solver/rpc:o");
    _stateMachine.setOutPointsPort(&_outPointsPort);
    _stateMachine.setOutTextPort(&_outTextPort);
    _stateMachine.setInFeaturesPort(&_inFeaturesPort);
    _stateMachine.setOutTtsPort(&_outTtsPort);
    _stateMachine.setInAsrPort(&_inAsrPort);
    _stateMachine.setFittingClient(&_fittingClient);
    _stateMachine.setGroundingClient(&_groundingClient);
    _stateMachine.setSolverClient(&_solverClient);
    while(1){
        if(_outTtsPort.getOutputCount() > 0) break;
        printf("Waiting for \"/dm1/tts:o\" to be connected to something...\n");
        Time::delay(0.5);
    }    
    _stateMachine.start();
    return true;
}

/************************************************************************/
double Dm1::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool Dm1::updateModule() {
    printf("StateMachine in state [%d]. Dm1 alive...\n", _stateMachine.getMachineState());
    return true;
}

/************************************************************************/

bool Dm1::interruptModule() {
    printf("Dm1 closing...\n");
    _inAsrPort.interrupt();
    _outTtsPort.interrupt();
    _fittingClient.interrupt();
    _groundingClient.interrupt();
    _solverClient.interrupt();
    _stateMachine.stop();
    _inAsrPort.close();
    _outTtsPort.close();
    _fittingClient.close();
    _solverClient.close();
    return true;
}

/************************************************************************/

