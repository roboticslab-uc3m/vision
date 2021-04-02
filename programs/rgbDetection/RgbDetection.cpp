// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include "RgbDetection.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_CAMERA_DEVICE "remote_grabber"
#define DEFAULT_CAMERA_LOCAL "/rgbDetection"
#define DEFAULT_CAMERA_REMOTE "/grabber"
#define DEFAULT_WATCHDOG    2       // [s]

/************************************************************************/

bool roboticslab::RgbDetection::configure(yarp::os::ResourceFinder &rf)
{
    cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strCameraDevice = DEFAULT_CAMERA_DEVICE;
    std::string strCameraLocal = DEFAULT_CAMERA_LOCAL;
    std::string strCameraRemote = DEFAULT_CAMERA_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    std::printf("RgbDetection options:\n");
    std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    std::printf("\t--cropSelector (default: \"%d\")\n", cropSelector);
    std::printf("\t--cameraDevice (device we create, default: \"%s\")\n", strCameraDevice.c_str());
    std::printf("\t--cameraLocal (if accesing remote, local port name, default: \"%s\")\n", strCameraLocal.c_str());
    std::printf("\t--cameraRemote (if accesing remote, remote port name, default: \"%s\")\n", strCameraRemote.c_str());
    std::printf("\t--watchdog ([s] default: \"%f\")\n", watchdog);

    if (rf.check("cropSelector"))
    {
        cropSelector = rf.find("cropSelector").asInt32();
    }

    yInfo() << "Using cropSelector:" << cropSelector;

    if (rf.check("cameraDevice"))
    {
        strCameraDevice = rf.find("cameraDevice").asString();
    }

    if (rf.check("cameraLocal"))
    {
        strCameraLocal = rf.find("cameraLocal").asString();
    }

    if (rf.check("cameraRemote"))
    {
        strCameraRemote = rf.find("cameraRemote").asString();
    }

    if (rf.check("watchdog"))
    {
        watchdog = rf.find("watchdog").asFloat64();
    }

    strCameraLocal ="/rgbDetection";

    yInfo("Using cameraDevice: %s, cameraLocal: %s, cameraRemote: %s",
        strCameraDevice.c_str(), strCameraLocal.c_str(), strCameraRemote.c_str());
    yInfo() << "Using watchdog:" << watchdog;


    yarp::os::Property options;
    options.fromString(rf.toString());
    options.put("device", strCameraDevice);
    options.put("local", strCameraLocal);
    options.put("remote", strCameraRemote);

    if(!cameraDevice.open(options))
    {
        yWarning() << "Bad camera open";
        return false;
    }
    yInfo() << "Camera device open (connection not assured, read YARP output above)";

    if(!cameraDevice.isValid())
    {
        yWarning() << "Camera not valid";
        return false;
    }
    yInfo() << "Camera device valid";

    if (!cameraDevice.view(camera))
    {
        yError() << "Camera device bad view";
        return false;
    }
    yInfo() << "Camera device ok view";

    detectorThread.setIFrameGrabberImageDriver(camera);
    detectorThread.setOutImg(&outImg);
    detectorThread.setOutPort(&outPort);
    detectorThread.setCropSelector(cropSelector);

    if (cropSelector != 0)
    {
        detectorThread.setOutCropSelectorImg(&outCropSelectorImg);
        detectorThread.setInCropSelectorPort(&inCropSelectorPort);
    }

    //-----------------OPEN LOCAL PORTS------------//

    std::string portPrefix("/rgbDetection");
    portPrefix += strCameraRemote;
    outImg.open(portPrefix + "/img:o");
    outPort.open(portPrefix + "/state:o");


    if (cropSelector != 0)
    {
        outCropSelectorImg.open(strCameraLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strCameraLocal + "/cropSelector/state:i");
    }

    return detectorThread.init(rf);
}

/*****************************************************************/

double roboticslab::RgbDetection::getPeriod()
{
    return watchdog;  // [s]
}

/************************************************************************/

bool roboticslab::RgbDetection::updateModule()
{
    yInfo() << "RgbDetection alive...";
    return true;
}

/************************************************************************/

bool roboticslab::RgbDetection::interruptModule()
{
    detectorThread.askToStop();

    outImg.interrupt();
    outPort.interrupt();

    if (cropSelector != 0)
    {
        outCropSelectorImg.interrupt();
        inCropSelectorPort.interrupt();
    }

    return true;
}

/************************************************************************/

bool roboticslab::RgbDetection::close()
{
    if (detectorThread.isRunning())
    {
        detectorThread.stop();
    }

    cameraDevice.close();
    outImg.close();
    outPort.close();

    if (cropSelector != 0)
    {
        outCropSelectorImg.close();
        inCropSelectorPort.close();
    }

    return true;
}

/************************************************************************/
