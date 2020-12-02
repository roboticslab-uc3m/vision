// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <ColorDebug.h>

#include "SwitchDetection.hpp"

namespace roboticslab
{

/************************************************************************/
bool SwitchDetection::configure(yarp::os::ResourceFinder &rf) {

    cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strRGBDDevice = DEFAULT_RGBD_DEVICE;
    std::string strRGBDLocal = DEFAULT_RGBD_LOCAL;
    std::string strRGBDRemote = DEFAULT_RGBD_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    printf("SwitchDetection options:\n");
    printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    printf("\t--cropSelector (default: \"%d\")\n",cropSelector);
    printf("\t--RGBDDevice (device we create, default: \"%s\")\n",strRGBDDevice.c_str());
    printf("\t--RGBDLocal (if accesing remote, local port name, default: \"%s\")\n",strRGBDLocal.c_str());
    printf("\t--RGBDRemote (if accesing remote, remote port name, default: \"%s\")\n",strRGBDRemote.c_str());
    printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);

    if(rf.check("cropSelector")) cropSelector = rf.find("cropSelector").asInt32();
    printf("SwitchDetection using cropSelector: %d.\n",cropSelector);
    if(rf.check("RGBDDevice")) strRGBDDevice = rf.find("RGBDDevice").asString();
    if(rf.check("RGBDLocal")) strRGBDLocal = rf.find("RGBDLocal").asString();
    if(rf.check("RGBDRemote")) strRGBDRemote = rf.find("RGBDRemote").asString();
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asFloat64();

    printf("SwitchDetection using RGBDDevice: %s, RGBDLocal: %s, RGBDRemote: %s.\n",
        strRGBDDevice.c_str(), strRGBDLocal.c_str(), strRGBDRemote.c_str());
    printf("SwitchDetection using watchdog: %f.\n",watchdog);

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
        CD_ERROR("Bad RGBDDevice \"%s\"...\n",strRGBDDevice.c_str());
        return false;
    }
    CD_SUCCESS("RGBDDevice available.\n");

    if (! dd.view(iRGBDSensor) )
    {
        CD_ERROR("RGBDDevice bad view.\n");
        return false;
    }
    CD_SUCCESS("RGBDDevice ok view.\n");

    segmentorThread.setIRGBDSensor(iRGBDSensor);
    segmentorThread.setOutImg(&outImg);
    segmentorThread.setOutPort(&outPort);

    segmentorThread.setCropSelector(cropSelector);
    if(cropSelector != 0) {
        segmentorThread.setOutCropSelectorImg(&outCropSelectorImg);
        segmentorThread.setInCropSelectorPort(&inCropSelectorPort);
    }

    //-----------------OPEN LOCAL PORTS------------//
    if(!outImg.open(strRGBDLocal + "/img:o"))
    {
        CD_ERROR("Bad outImg.open\n");
        return false;
    }
    if(!outPort.open(strRGBDLocal + "/state:o"))
    {
        CD_ERROR("Bad outPort.open\n");
        return false;
    }
    yarp::os::Time::delay(1);
    if(cropSelector != 0)
    {
        outCropSelectorImg.open(strRGBDLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strRGBDLocal + "/cropSelector/state:i");
    }

    if(!segmentorThread.init(rf))
    {
        CD_ERROR("Bad segmentorThread.init\n");
        return false;
    }
    CD_SUCCESS("\n");
    return true;
}

/*****************************************************************/
double SwitchDetection::getPeriod() {
    return watchdog;  // [s]
}

/************************************************************************/

bool SwitchDetection::updateModule() {
    printf("SwitchDetection alive...\n");
    return true;
}

/************************************************************************/

bool SwitchDetection::interruptModule() {
    printf("SwitchDetection closing...\n");
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
