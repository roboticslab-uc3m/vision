// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*

#include <yarp/sig/ImageDraw.h>

#include <ColorDebug.h>

#include "DetectorThread.hpp"

#define DEFAULT_DETECTOR "HaarDetector"

/************************************************************************/

void roboticslab::DetectorThread::setIFrameGrabberImageDriver(yarp::dev::IFrameGrabberImage *_camera)
{
    camera = _camera;
}

/************************************************************************/

void roboticslab::DetectorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/

void roboticslab::DetectorThread::setOutPort(yarp::os::Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/

bool roboticslab::DetectorThread::init(yarp::os::ResourceFinder &rf)
{
    int rateMs = DEFAULT_RATE_MS;

    std::printf("DetectorThread options:\n");
    std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    std::printf("\t--rateMs (default: \"%d\")\n", rateMs);
    std::printf("\t--detector (default: \"%s\")\n", DEFAULT_DETECTOR);

    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt32();
    }

    yarp::os::Property deviceOptions;
    deviceOptions.fromString(rf.toString());

    deviceOptions.put("device", deviceOptions.check("detector", yarp::os::Value(DEFAULT_DETECTOR), "detector to be used"));

    if(!detectorDevice.open(deviceOptions))
    {
        CD_ERROR("Failed to open device: %s\n", rf.find("device").toString().c_str());
        return false;
    }

    if (!detectorDevice.view(iDetector))
    {
        CD_ERROR("Problems acquiring detector interface\n");
        return false;
    }

    CD_SUCCESS("Acquired detector interface\n");

    if (cropSelector != 0)
    {
        cropSelectorProcessor.reset();
        inCropSelectorPort->setReader(cropSelectorProcessor);
    }

    if(!setPeriod(rateMs * 0.001))
    {
        CD_ERROR("\n");
        return false;
    }

    if(!start())
    {
        CD_ERROR("\n");
        return false;
    }

    return true;
}

/************************************************************************/

void roboticslab::DetectorThread::run()
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;
    if (!camera->getImage(inYarpImg))
    {
        return;
    }

    //CD_DEBUG("\n");

    yarp::sig::VectorOf<DetectedObject> detectedObjects;
    yarp::os::Bottle output;

    bool ok = iDetector->detect(inYarpImg, detectedObjects);

    if(!ok)
    {
        CD_WARNING("Detector failed!\n");
        return;
    }

    // paint on image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg = inYarpImg;
    yarp::sig::PixelRgb red(255, 0, 0);
    for (size_t i = 0; i < detectedObjects.size(); i++)
    {
        int cx = (detectedObjects[0]._tlx + detectedObjects[0]._brx) / 2;
        int cy = (detectedObjects[0]._tly + detectedObjects[0]._bry) / 2;
        int width = detectedObjects[0]._brx - detectedObjects[0]._tlx;
        int height = detectedObjects[0]._bry - detectedObjects[0]._tly;

        yarp::sig::draw::addRectangleOutline(outYarpImg,
                                             red,
                                             cx,
                                             cy,
                                             width / 2,
                                             height / 2);

        yarp::os::Bottle b;
        b.addInt32(detectedObjects[i]._tlx);
        b.addInt32(detectedObjects[i]._tly);
        b.addInt32(detectedObjects[i]._brx);
        b.addInt32(detectedObjects[i]._bry);
        output.addList() = b;
    }

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

    if (output.size() > 0)
    {
        pOutPort->write(output);
    }
}
