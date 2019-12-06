// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*

#include <yarp/os/Bottle.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

#include <ColorDebug.h>

#include "DetectorThread.hpp"

#include "ColorRegionDetector.hpp"
#include "HaarDetector.hpp"
#include "TensorFlowDetector.hpp"

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

    if(detectorName=="Haar")
    {
        detector = new HaarDetector(&rf);
    }
    else if(detectorName=="ColorRegion")
    {
        detector = new ColorRegionDetector(&rf);
    }
    else if(detectorName=="TensorFlow")
    {
        detector = new TensorFlowDetector(&rf);
    }
    else
    {
        CD_ERROR("\"detector\" not allowed (available: Haar, ColorRegion, TensorFlow): %s\n", detectorName.c_str());
        return false;
    }

    if(!detector->isValid())
    {
        CD_ERROR("\n");
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

    CD_DEBUG("\n");

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
    std::vector<BoundingBox> bbs;
    yarp::os::Bottle output;
    bool ok = detector->detect(inYarpImg, bbs, outYarpImg);

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
