// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <ColorDebug.h>

#include "SwitchDetection2D.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_CAMERA_DEVICE "remote_grabber"
#define DEFAULT_CAMERA_LOCAL "/switchDetection2D"
#define DEFAULT_CAMERA_REMOTE "/grabber"
#define DEFAULT_WATCHDOG    2       // [s]

/************************************************************************/

bool roboticslab::SwitchDetection2D::configure(yarp::os::ResourceFinder &rf)
{
    cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strCameraDevice = DEFAULT_CAMERA_DEVICE;
    std::string strCameraLocal = DEFAULT_CAMERA_LOCAL;
    std::string strCameraRemote = DEFAULT_CAMERA_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    std::printf("SwitchDetection2D options:\n");
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

    CD_INFO("Using cropSelector: %d.\n", cropSelector);

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

    strCameraLocal ="/switchDetection2D";

    CD_INFO("Using cameraDevice: %s, cameraLocal: %s, cameraRemote: %s.\n",
        strCameraDevice.c_str(), strCameraLocal.c_str(), strCameraRemote.c_str());
    CD_INFO("Using watchdog: %f.\n", watchdog);


    yarp::os::Property options;
    options.fromString(rf.toString());
    options.put("device", strCameraDevice);
    options.put("local", strCameraLocal);
    options.put("remote", strCameraRemote);

    if(!cameraDevice.open(options))
    {
        CD_WARNING("Bad camera open.\n");
        return false;
    }
    CD_SUCCESS("Camera device open (connection not assured, read YARP output above).\n");

    if(!cameraDevice.isValid())
    {
        CD_WARNING("Camera not valid\n");
        return false;
    }
    CD_SUCCESS("Camera device valid.\n");

    if (!cameraDevice.view(camera))
    {
        CD_ERROR("Camera device bad view.\n");
        return false;
    }
    CD_SUCCESS("Camera device ok view.\n");

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

    std::string portPrefix("/switchDetection2D");
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

double roboticslab::SwitchDetection2D::getPeriod()
{
    return watchdog;  // [s]
}

/************************************************************************/

bool roboticslab::SwitchDetection2D::updateModule()
{
    CD_INFO("Alive...\n");
    return true;
}

/************************************************************************/

bool roboticslab::SwitchDetection2D::interruptModule()
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

bool roboticslab::SwitchDetection2D::close()
{
    CD_INFO("Closing...\n");

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
