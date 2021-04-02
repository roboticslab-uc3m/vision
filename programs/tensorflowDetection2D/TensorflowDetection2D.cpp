// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TensorflowDetection2D.hpp"
#include "TensorflowDetector.hpp"
#include "SegmentorThread.hpp"

#include <cstdio>
#include <string>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

using namespace roboticslab;

/************************************************************************/

bool TensorflowDetection2D::configure(yarp::os::ResourceFinder &rf)
{
    cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strCameraDevice = DEFAULT_CAMERA_DEVICE;
    std::string strCameraLocal = DEFAULT_CAMERA_LOCAL;
    std::string strCameraRemote = DEFAULT_CAMERA_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    if (rf.check("help"))
    {
        std::printf("TensorflowDetection2D options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        std::printf("\t--cropSelector (default: \"%d\")\n", cropSelector);
        std::printf("\t--cameraDevice (device we create, default: \"%s\")\n", strCameraDevice.c_str());
        std::printf("\t--cameraLocal (if accesing remote, local port name, default: \"%s\")\n", strCameraLocal.c_str());
        std::printf("\t--cameraRemote (if accesing remote, remote port name, default: \"%s\")\n", strCameraRemote.c_str());
        std::printf("\t--watchdog ([s] default: \"%f\")\n", watchdog);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("cropSelector"))
    {
        cropSelector = rf.find("cropSelector").asInt();
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
        watchdog = rf.find("watchdog").asDouble();
    }

    yInfo("Using cameraDevice: %s, cameraLocal: %s, cameraRemote: %s",
        strCameraDevice.c_str(), strCameraLocal.c_str(), strCameraRemote.c_str());

    yInfo() << "Using watchdog:" << watchdog;

    if (!rf.check("help"))
    {
        yarp::os::Property options;
        options.fromString(rf.toString());
        options.put("device", strCameraDevice);
        options.put("local", strCameraLocal);
        options.put("remote", strCameraRemote);

        while (!dd.open(options))
        {
            yInfo("Waiting for camera device \"%s\"...", strCameraDevice.c_str());
            yarp::os::Time::delay(1);
        }

        yInfo() << "Camera device available";

        if (!dd.view(camera))
        {
            yWarning() << "Bad view of camera from device";
        }
        else
        {
            yInfo() << "Ok view of camera from device";
        }

    /*    if (!dd.view(iRgbVisualParams))
        {
            yWarning() << "Bad view of iRgbVisualParams from device";
        }
        else
        {
            yInfo() << "Ok view of iRgbVisualParams from device";
        }

        segmentorThread.setH(iRgbVisualParams->getRgbHeight());
        segmentorThread.setW(iRgbVisualParams->getRgbWidth());*/

        segmentorThread.setIFrameGrabberImageDriver(camera);
        segmentorThread.setOutImg(&outImg);
        segmentorThread.setOutPort(&outPort);
        segmentorThread.setCropSelector(cropSelector);

        if (cropSelector != 0)
        {
            segmentorThread.setOutCropSelectorImg(&outCropSelectorImg);
            segmentorThread.setInCropSelectorPort(&inCropSelectorPort);
        }
    }

    segmentorThread.init(rf);

    //-----------------OPEN LOCAL PORTS------------//

    outImg.open(strCameraLocal + "/img:o");
    outPort.open(strCameraLocal + "/state:o");

    if (cropSelector != 0)
    {
        outCropSelectorImg.open(strCameraLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strCameraLocal + "/cropSelector/state:i");
    }

    return true;
}

/*****************************************************************/

double TensorflowDetection2D::getPeriod()
{
    return watchdog;  // [s]
}

/************************************************************************/

bool TensorflowDetection2D::updateModule()
{
    yInfo() << "TensorflowDetection2D alive...";
    return true;
}

/************************************************************************/

bool TensorflowDetection2D::interruptModule()
{
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

bool TensorflowDetection2D::close()
{
    segmentorThread.stop();

    dd.close();
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
