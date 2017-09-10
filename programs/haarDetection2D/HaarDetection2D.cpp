// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetection2D.hpp"

#include <cstdio>
#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

using namespace roboticslab;

/************************************************************************/

bool HaarDetection2D::configure(yarp::os::ResourceFinder &rf)
{
    cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strKinectDevice = DEFAULT_KINECT_DEVICE;
    std::string strKinectLocal = DEFAULT_KINECT_LOCAL;
    std::string strKinectRemote = DEFAULT_KINECT_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    std::fprintf(stdout,"--------------------------------------------------------------\n");

    if (rf.check("help"))
    {
        std::printf("HaarDetection2D options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        std::printf("\t--cropSelector (default: \"%d\")\n", cropSelector);
        std::printf("\t--visionDevice (device we create, default: \"%s\")\n", strKinectDevice.c_str());
        std::printf("\t--visionLocal (if accesing remote, local port name, default: \"%s\")\n", strKinectLocal.c_str());
        std::printf("\t--visionRemote (if accesing remote, remote port name, default: \"%s\")\n", strKinectRemote.c_str());
        std::printf("\t--watchdog ([s] default: \"%f\")\n", watchdog);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("cropSelector"))
    {
        cropSelector = rf.find("cropSelector").asInt();
    }

    std::printf("HaarDetection2D using cropSelector: %d.\n", cropSelector);

    if (rf.check("kinectDevice"))
    {
        strKinectDevice = rf.find("kinectDevice").asString();
    }

    if (rf.check("kinectLocal"))
    {
        strKinectLocal = rf.find("kinectLocal").asString();
    }

    if (rf.check("kinectRemote"))
    {
        strKinectRemote = rf.find("kinectRemote").asString();
    }

    if (rf.check("watchdog"))
    {
        watchdog = rf.find("watchdog").asDouble();
    }

    std::printf("HaarDetection2D using kinectDevice: %s, kinectLocal: %s, kinectRemote: %s.\n",
        strKinectDevice.c_str(), strKinectLocal.c_str(), strKinectRemote.c_str());

    std::printf("HaarDetection2D using watchdog: %f.\n", watchdog);

    if (!rf.check("help"))
    {
        yarp::os::Property options;
        options.fromString(rf.toString());  //-- Should get noMirror, noRGBMirror, noDepthMirror, video modes...
        options.put("device", strKinectDevice);  //-- Important to override in case there is a "device" in the future
        options.put("localName", strKinectLocal);  //
        options.put("remoteName", strKinectRemote);  //
        //if(rf.check("noMirror")) options.put("noMirror",1);  //-- Replaced by options.fromString( rf.toString() );

        while (!dd.open(options))
        {
            std::printf("Waiting for kinectDevice \"%s\"...\n", strKinectDevice.c_str());
            yarp::os::Time::delay(1);
        }

        std::printf("[HaarDetection2D] success: kinectDevice available.\n");

        if (!dd.view(kinect))
        {
            std::fprintf(stderr, "[HaarDetection2D] warning: kinectDevice bad view.\n");
        }
        else
        {
            std::printf("[HaarDetection2D] success: kinectDevice ok view.\n");
        }

        segmentorThread.setIKinectDeviceDriver(kinect);
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

    outImg.open(strKinectLocal + "/img:o");
    outPort.open(strKinectLocal + "/state:o");

    if (cropSelector != 0)
    {
        outCropSelectorImg.open(strKinectLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strKinectLocal + "/cropSelector/state:i");
    }

    return true;
}

/*****************************************************************/

double HaarDetection2D::getPeriod()
{
    return watchdog;  // [s]
}

/************************************************************************/

bool HaarDetection2D::updateModule()
{
    std::printf("HaarDetection2D alive...\n");
    return true;
}

/************************************************************************/

bool HaarDetection2D::interruptModule()
{
    std::printf("HaarDetection2D closing...\n");

    segmentorThread.stop();
    outImg.interrupt();
    outPort.interrupt();

    if (cropSelector != 0)
    {
        outCropSelectorImg.interrupt();
        inCropSelectorPort.interrupt();
    }

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
