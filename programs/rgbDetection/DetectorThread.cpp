// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*

#include <yarp/os/LogStream.h>
#include <yarp/sig/ImageDraw.h>

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
        yError() << "Failed to open device:" << rf.find("device").toString();
        return false;
    }

    if (!detectorDevice.view(iDetector))
    {
        yError() << "Problems acquiring detector interface";
        return false;
    }

    if (cropSelector != 0)
    {
        cropSelectorProcessor.reset();
        inCropSelectorPort->setReader(cropSelectorProcessor);
    }

    if (!setPeriod(rateMs * 0.001))
    {
        yError() << "Unable to set period";
        return false;
    }

    if (!start())
    {
        yError() << "Thread could not start";
        return false;
    }

    return true;
}

/************************************************************************/

void roboticslab::DetectorThread::run()
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImgRgb;

    if (!camera->getImage(inYarpImgRgb))
    {
        return;
    }

    std::vector<yarp::os::Property> detectedObjects;

    if (!iDetector->detect(inYarpImgRgb, detectedObjects))
    {
        yWarning() << "Detector failed!";
        return;
    }

    // paint on image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg = inYarpImgRgb;
    yarp::sig::PixelRgb red(255, 0, 0);
    yarp::os::Bottle output;

    for (size_t i = 0; i < detectedObjects.size(); i++)
    {
        int cx = (detectedObjects[i].find("tlx").asInt32() + detectedObjects[i].find("brx").asInt32()) / 2;
        int cy = (detectedObjects[i].find("tly").asInt32() + detectedObjects[i].find("bry").asInt32()) / 2;
        int width = detectedObjects[i].find("brx").asInt32() - detectedObjects[i].find("tlx").asInt32();
        int height = detectedObjects[i].find("bry").asInt32() - detectedObjects[i].find("tly").asInt32();

        yarp::sig::draw::addRectangleOutline(outYarpImg,
                                             red,
                                             cx,
                                             cy,
                                             width / 2,
                                             height / 2);

        output.addDict() = detectedObjects[i];
    }

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

    if (output.size() > 0)
    {
        pOutPort->write(output);
    }
}
