// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ColorDebug.h>

#include "SegmentorThread.hpp"

#include "HaarDetection2D.hpp"

#define DEFAULT_SWITCH_MODE "haarDetection"

/************************************************************************/

void roboticslab::SegmentorThread::setIFrameGrabberImageDriver(yarp::dev::IFrameGrabberImage *_camera)
{
    camera = _camera;
}

/************************************************************************/

void roboticslab::SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/

void roboticslab::SegmentorThread::setOutPort(yarp::os::Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/
//TensorflowDetection2D tensorflowDetector;

bool roboticslab::SegmentorThread::init(yarp::os::ResourceFinder &rf)
{
    int rateMs = DEFAULT_RATE_MS;
    std::string switchMode = DEFAULT_SWITCH_MODE;

    std::printf("SegmentorThread options:\n");
    std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    std::printf("\t--rateMs (default: \"%d\")\n", rateMs);

    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt32();
    }

    if (rf.check("switchMode"))
    {
        switchMode = rf.find("switchMode").asString();
    }
    CD_INFO("Using switchMode: %s\n", switchMode.c_str());

    if(switchMode=="haarDetection")
    {
        transformation = new HaarDetectionTransformation(&rf);
        if(!transformation->isValid())
        {
            CD_ERROR("\n");
            return false;
        }
    }
    else if(switchMode=="colorRegionDetection")
    {

    }
    else if(switchMode=="tensorflowDetection")
    {
    }
    else
    {
        CD_ERROR("switchMode not allowed (available: haarDetection, colorRegionDetection, tensorflowDetection): %s\n", switchMode.c_str());
        return false;
    }

    if (cropSelector != 0)
    {
        cropSelectorProcessor.reset();
        inCropSelectorPort->setReader(cropSelectorProcessor);
    }

    setPeriod(rateMs * 0.001);
    start();

    return true;
}

/************************************************************************/

void roboticslab::SegmentorThread::run()
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
    yarp::os::Bottle output;

    if (!camera->getImage(inYarpImg))
    {
        return;
    }

    CD_DEBUG("Executing detection...\n");
    outYarpImg = transformation->detect(inYarpImg);

    /*
    else if(strSwitchMode=="colorRegionDetection")
    {
        std::cout<<"Executing ColorRegionDetection2D..."<<std::endl;
        ColorRegionDetection2D colorRegionDetector;
        //    /outYarpImg=/
        colorRegionDetector.run(inYarpImg, algorithm, locate, morphClosing, maxNumBlobs,threshold);
        outYarpImg=colorRegionDetector.outImageProcessed;
        output=colorRegionDetector.outputProcessed;
    }
    else if(strSwitchMode=="tensorflowDetection")
    {
        std::cout<<"Ejecutando tensorflowDetection2D"<<std::endl;
        outYarpImg=tensorflowDetector.run(inYarpImg);
        output=tensorflowDetector.bottle;
    }
    */

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

    if (output.size() > 0)
    {
        pOutPort->write(output);
    }
}
