// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OpencvDnnDetection.hpp"

namespace roboticslab
{

/************************************************************************/
bool OpencvDnnDetection::configure(yarp::os::ResourceFinder &rf) {

    cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strRGBDDevice = DEFAULT_RGBD_DEVICE;
    std::string strRGBDLocal = DEFAULT_RGBD_LOCAL;
    std::string strRGBDRemote = DEFAULT_RGBD_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    printf("OpencvDnnDetection options:\n");
    printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    printf("\t--cropSelector (default: \"%d\")\n",cropSelector);
    printf("\t--RGBDDevice (device we create, default: \"%s\")\n",strRGBDDevice.c_str());
    printf("\t--RGBDLocal (if accesing remote, local port name, default: \"%s\")\n",strRGBDLocal.c_str());
    printf("\t--RGBDRemote (if accesing remote, remote port name, default: \"%s\")\n",strRGBDRemote.c_str());
    printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);

    if(rf.check("cropSelector")) cropSelector = rf.find("cropSelector").asInt32();
    printf("RgbdDetection using cropSelector: %d.\n",cropSelector);
    if(rf.check("RGBDDevice")) strRGBDDevice = rf.find("RGBDDevice").asString();
    if(rf.check("RGBDLocal")) strRGBDLocal = rf.find("RGBDLocal").asString();
    if(rf.check("RGBDRemote")) strRGBDRemote = rf.find("RGBDRemote").asString();
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asFloat64();

    printf("OpencvDnnDetection using RGBDDevice: %s, RGBDLocal: %s, RGBDRemote: %s.\n",
        strRGBDDevice.c_str(), strRGBDLocal.c_str(), strRGBDRemote.c_str());
    printf("OpencvDnnDetection using watchdog: %f.\n",watchdog);

    yarp::os::Property options;
    options.fromString( rf.toString() );  //-- Should get noMirror, noRGBMirror, noDepthMirror, video modes...
    options.put("device",strRGBDDevice);  //-- Important to override in case there is a "device" in the future
    options.put("localImagePort",strRGBDLocal+"/rgbImage:i");
    options.put("localDepthPort",strRGBDLocal+"/depthImage:i");
    options.put("localRpcPort",strRGBDLocal+"/rpc:o");
    options.put("remoteImagePort",strRGBDRemote+"/rgbImage:o");
    options.put("remoteDepthPort",strRGBDRemote+"/depthImage:o");
    options.put("remoteRpcPort",strRGBDRemote+"/rpc:i");
    //if(rf.check("noMirror")) options.put("noMirror",1);  //-- Replaced by options.fromString( rf.toString() );

    if(!dd.open(options))
    {
        yError("Bad RGBDDevice \"%s\"...\n",strRGBDDevice.c_str());
        return false;
    }
    yInfo("RGBDDevice available.\n");

    if (! dd.view(iRGBDSensor) )
    {
        yError("RGBDDevice bad view.\n");
        return false;
    }
    yInfo("RGBDDevice ok view.\n");

    segmentorThread.setIRGBDSensor(iRGBDSensor);
    segmentorThread.setOutImg(&outImg);
    segmentorThread.setOutPort(&outPort);

    segmentorThread.setCropSelector(cropSelector);
    if(cropSelector != 0) {
        segmentorThread.setOutCropSelectorImg(&outCropSelectorImg);
        segmentorThread.setInCropSelectorPort(&inCropSelectorPort);
    }

    //-----------------OPEN LOCAL PORTS------------//
    std::string portPrefix("/opencvDnnDetection");
    portPrefix += strRGBDRemote;
    if(!outImg.open(portPrefix + "/img:o"))
    {
        yError("Bad outImg.open\n");
        return false;
    }
    if(!outPort.open(portPrefix + "/detections:o"))
    {
        yError("Bad outPort.open\n");
        return false;
    }

    if(cropSelector != 0)
    {
        outCropSelectorImg.open(strRGBDLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strRGBDLocal + "/cropSelector/state:i");
    }

    if(!segmentorThread.init(rf))
    {
        yError("Bad segmentorThread.init\n");
        return false;
    }
    yInfo("--- end: configure\n");
    return true;
}

/*****************************************************************/
double OpencvDnnDetection::getPeriod() {
    return watchdog;  // [s]
}

/************************************************************************/

bool OpencvDnnDetection::updateModule() {
    printf("OpencvDnnDetection alive...\n");
    return true;
}

/************************************************************************/

bool OpencvDnnDetection::interruptModule() {
    printf("OpencvDnnDetection closing...\n");
    segmentorThread.stop();
    outImg.interrupt();
    outPort.interrupt();
    if(cropSelector != 0) {
        outCropSelectorImg.interrupt();
        inCropSelectorPort.interrupt();
    }
    dd.close();
    outImg.close();
    outPort.close();
    if(cropSelector != 0) {
        outCropSelectorImg.close();
        inCropSelectorPort.close();
    }
    return true;
}

/************************************************************************/

}  // namespace roboticslab
