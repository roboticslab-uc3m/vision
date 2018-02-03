// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinectPxToReal.hpp"

/************************************************************************/
bool KinectPxToReal::updateModule() {
    printf("KinectPxToReal alive...\n");
    return true;
}

/************************************************************************/
double KinectPxToReal::getPeriod() {
    return watchdog;  // [s]
}

/************************************************************************/

bool KinectPxToReal::configure(ResourceFinder &rf) {
    double fx = DEFAULT_FX;
    double fy = DEFAULT_FY;
    double cx = DEFAULT_CX;
    double cy = DEFAULT_CY;
    watchdog = DEFAULT_WATCHDOG;  // double

    fprintf(stdout,"--------------------------------------------------------------\n");
    if(rf.check("help")) {
       printf("KinectPxToReal Options:\n");
       printf("\t--fx (default: \"%f\")\n",fx);
       printf("\t--fy (default: \"%f\")\n",fy);
       printf("\t--cx (default: \"%f\")\n",cx);
       printf("\t--cy (default: \"%f\")\n",cy);
       printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);
    }
    if(rf.check("fx")) fx = rf.find("fx").asDouble();
    if(rf.check("fy")) fy = rf.find("fy").asDouble();
    if(rf.check("cx")) cx = rf.find("cx").asDouble();
    if(rf.check("cy")) cy = rf.find("cy").asDouble();
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asDouble();
    fprintf(stdout,"KinectPxToReal using fx: %f, fy: %f, cx: %f, cy: %f.\n",fx,fy,cx,cy);
    fprintf(stdout,"KinectPxToReal using watchdog [s]: %f.\n",watchdog);
    fprintf(stdout,"--------------------------------------------------------------\n");
    if(rf.check("help")) {
       return false;
    }

    callbackPort.setParams(fx,fy,cx,cy);
    callbackPort.setDepthPort(&depthPort);
    callbackPort.setOutPort(&outPort);
    
    //-----------------OPEN LOCAL PORTS------------//
    depthPort.open("/kinectPxToReal/depth:i");
    outPort.open("/kinectPxToReal/state:o");
    callbackPort.open("/kinectPxToReal/state:i");

    callbackPort.useCallback();

    return true;
}

/************************************************************************/

bool KinectPxToReal::interruptModule() {
    callbackPort.disableCallback();
    callbackPort.interrupt();
    depthPort.interrupt();
    outPort.interrupt();
    callbackPort.close();
    depthPort.close();
    outPort.close();
    return true;
}

/************************************************************************/

