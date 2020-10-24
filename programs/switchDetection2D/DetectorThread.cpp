// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*

#include <yarp/sig/ImageDraw.h>

#include <ColorDebug.h>

#include "DetectorThread.hpp"

#define DEFAULT_DETECTOR "Haar"

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
//TensorflowDetection2D tensorflowDetector;

bool roboticslab::DetectorThread::init(yarp::os::ResourceFinder &rf)
{
    int rateMs = DEFAULT_RATE_MS;
    std::string detectorName = DEFAULT_DETECTOR;

    std::printf("DetectorThread options:\n");
    std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    std::printf("\t--rateMs (default: \"%d\")\n", rateMs);

    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt32();
    }

    if (rf.check("detector"))
    {
        detectorName = rf.find("detector").asString();
    }
    CD_INFO("Using detector: %s\n", detectorName.c_str());

    yarp::os::Property detectorOptions;
    detectorOptions.put("device", detectorName);

    if(!detector->open(detectorOptions))
    {
        CD_ERROR("\"detector\" not allowed (available: Haar, ColorRegion, TensorFlow): %s\n", detectorName.c_str());
        return false;
    }

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

    bool ok = detector->detect(inYarpImg, detectedObjects);

    // paint on image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg = inYarpImg;
    yarp::sig::PixelRgb red(255, 0, 0);
    for (size_t i = 0; i < detectedObjects.size(); i++)
    {
        yarp::sig::draw::addRectangleOutline(outYarpImg,
                                             red,
                                             detectedObjects[i].cx(),
                                             detectedObjects[i].cy(),
                                             detectedObjects[i].width() / 2,
                                             detectedObjects[i].height() / 2);
    }

    /*
    colorRegionDetector.run(inYarpImg, algorithm, locate, morphClosing, maxNumBlobs,threshold);
    outYarpImg=colorRegionDetector.outImageProcessed;
    outYarpImg=tensorflowDetector.run(inYarpImg);
    output=colorRegionDetector.outputProcessed;
    output=tensorflowDetector.bottle;
    */

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

    if (output.size() > 0)
    {
        pOutPort->write(output);
    }
}
