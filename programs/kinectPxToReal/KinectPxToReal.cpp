// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinectPxToReal.hpp"

#include <string>

#include <yarp/os/Property.h>

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

bool KinectPxToReal::configure(yarp::os::ResourceFinder &rf) {
    std::string strRGBDDevice = DEFAULT_RGBD_DEVICE;
    std::string strRGBDLocal = DEFAULT_RGBD_LOCAL;
    std::string strRGBDRemote = DEFAULT_RGBD_REMOTE;

    watchdog = DEFAULT_WATCHDOG;  // double

    fprintf(stdout,"--------------------------------------------------------------\n");

    if(rf.check("help")) {
       printf("KinectPxToReal Options:\n");
       printf("\t--RGBDDevice (default: \"%s\")\n",strRGBDDevice.c_str());
       printf("\t--RGBDLocal (default: \"%s\")\n",strRGBDLocal.c_str());
       printf("\t--RGBDRemote (default: \"%s\")\n",strRGBDRemote.c_str());
       printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);
    }

    if(rf.check("RGBDDevice")) strRGBDDevice = rf.find("RGBDDevice").asString();
    if(rf.check("RGBDLocal")) strRGBDLocal = rf.find("RGBDLocal").asString();
    if(rf.check("RGBDRemote")) strRGBDRemote = rf.find("RGBDRemote").asString();
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asDouble();

    fprintf(stdout,"KinectPxToReal using watchdog [s]: %f.\n",watchdog);
    fprintf(stdout,"--------------------------------------------------------------\n");

    if(rf.check("help")) {
       return false;
    }

    yarp::os::Property rgbdOptions;
    rgbdOptions.put("device", strRGBDDevice);
    rgbdOptions.put("localImagePort", strRGBDLocal + "/rgbImage:i");
    rgbdOptions.put("localDepthPort", strRGBDLocal + "/depthImage:i");
    rgbdOptions.put("localRpcPort", strRGBDLocal + "/rpc:o");
    rgbdOptions.put("remoteImagePort", strRGBDRemote + "/rgbImage:o");
    rgbdOptions.put("remoteDepthPort", strRGBDRemote + "/depthImage:o");
    rgbdOptions.put("remoteRpcPort", strRGBDRemote + "/rpc:i");

    if (!rgbdDevice.open(rgbdOptions)) {
        fprintf(stderr, "cannot open device: %s\n", strRGBDDevice.c_str());
        return false;
    }

    if (!rgbdDevice.view(irgbdSensor)) {
        fprintf(stderr, "cannot view irgbdSensor\n");
        return false;
    }

    yarp::os::Property depthIntrinsicParams;

    if (!irgbdSensor->getDepthIntrinsicParam(depthIntrinsicParams)) {
        fprintf(stderr, "cannot retrieve depth intrinsic parameters\n");
        return false;
    }

    double fx = depthIntrinsicParams.find("focalLengthX").asFloat64();;
    double fy = depthIntrinsicParams.find("focalLengthY").asFloat64();;
    double cx = depthIntrinsicParams.find("principalPointX").asFloat64();;
    double cy = depthIntrinsicParams.find("principalPointY").asFloat64();;

    fprintf(stdout,"KinectPxToReal using fx: %f, fy: %f, cx: %f, cy: %f.\n",fx,fy,cx,cy);

    callbackPort.setParams(fx,fy,cx,cy);
    callbackPort.setDepthSensorHandle(irgbdSensor);
    callbackPort.setOutPort(&outPort);
    
    //-----------------OPEN LOCAL PORTS------------//
    outPort.open(strRGBDLocal + "/state:o");
    callbackPort.open(strRGBDLocal + "/state:i");

    callbackPort.useCallback();

    return true;
}

/************************************************************************/

bool KinectPxToReal::interruptModule() {
    callbackPort.disableCallback();
    callbackPort.interrupt();
    outPort.interrupt();
    callbackPort.close();
    outPort.close();
    rgbdDevice.close();
    return true;
}

/************************************************************************/
