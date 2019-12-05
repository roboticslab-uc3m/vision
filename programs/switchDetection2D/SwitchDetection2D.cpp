// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iostream>
#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <ColorDebug.h>
#include "SwitchDetection2D.hpp"

/************************************************************************/

bool roboticslab::SwitchDetection2D::configure(yarp::os::ResourceFinder &rf)
{
    cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strCameraDevice = DEFAULT_CAMERA_DEVICE;
    std::string strCameraLocal = DEFAULT_CAMERA_LOCAL;
    std::string strCameraRemote = DEFAULT_CAMERA_REMOTE;
    std::string strSwitchMode = DEFAULT_SWITCH_MODE;
    watchdog = DEFAULT_WATCHDOG;  // double

    std::printf("SwitchDetection2D options:\n");
    std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    std::printf("\t--cropSelector (default: \"%d\")\n", cropSelector);
    std::printf("\t--cameraDevice (device we create, default: \"%s\")\n", strCameraDevice.c_str());
    std::printf("\t--cameraLocal (if accesing remote, local port name, default: \"%s\")\n", strCameraLocal.c_str());
    std::printf("\t--cameraRemote (if accesing remote, remote port name, default: \"%s\")\n", strCameraRemote.c_str());
    std::printf("\t--switchMode (default: \"%s\")\n", strSwitchMode.c_str());
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

    if (rf.check("switchMode"))
    {
        strSwitchMode = rf.find("switchMode").asString();
        std::cout<<"el modo es: "<<strSwitchMode<<" mode"<<std::endl;

        if((strSwitchMode!="haarDetection")&&(strSwitchMode!="tensorflowDetection")&&(strSwitchMode!="colorRegionDetection"))
        {
            std::cout<<strSwitchMode<<" mode not allowed"<<std::endl;
            return false;
        }

        strCameraLocal ="/"+strSwitchMode+"2D";
        rf.setDefaultContext(strSwitchMode);
    }

    CD_INFO("Using cameraDevice: %s, cameraLocal: %s, cameraRemote: %s.\n",
        strCameraDevice.c_str(), strCameraLocal.c_str(), strCameraRemote.c_str());
    CD_INFO("Using switchMode: %s.\n", strSwitchMode.c_str());
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

    segmentorThread.setIFrameGrabberImageDriver(camera);
    segmentorThread.setOutImg(&outImg);
    segmentorThread.setOutPort(&outPort);
    segmentorThread.setCropSelector(cropSelector);

    if (cropSelector != 0)
    {
        segmentorThread.setOutCropSelectorImg(&outCropSelectorImg);
        segmentorThread.setInCropSelectorPort(&inCropSelectorPort);
    }

    //-----------------OPEN LOCAL PORTS------------//

    outImg.open(strCameraLocal + "/img:o");

    if((strSwitchMode=="colorRegionDetection"))
    {
        outPort.open(strCameraLocal + "/features:o");
    }
    else if((strSwitchMode=="tensorflowDetection"))
    {
        outPort.open(strCameraLocal + "/results:o");
    }
    else
    {
        outPort.open(strCameraLocal + "/state:o");
    }


    if (cropSelector != 0)
    {
        outCropSelectorImg.open(strCameraLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strCameraLocal + "/cropSelector/state:i");
    }

    return segmentorThread.init(rf);
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

    segmentorThread.stop();

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
