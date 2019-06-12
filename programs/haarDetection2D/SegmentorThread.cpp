// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ColorDebug.h>

using namespace roboticslab;

/************************************************************************/

void SegmentorThread::setIFrameGrabberImageDriver(yarp::dev::IFrameGrabberImage *_camera)
{
    camera = _camera;
}

/************************************************************************/

void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/

void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/

void SegmentorThread::init(yarp::os::ResourceFinder &rf)
{
    int rateMs = DEFAULT_RATE_MS;

    std::string xmlCascade = DEFAULT_XMLCASCADE;

    if (rf.check("help"))
    {
        std::printf("SegmentorThread options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        std::printf("\t--rateMs (default: \"%d\")\n", rateMs);
        std::printf("\t--xmlCascade [file.xml] (default: \"%s\")\n", xmlCascade.c_str());
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt();
    }

    if (rf.check("xmlCascade"))
    {
        xmlCascade = rf.find("xmlCascade").asString();
    }

    if (rf.check("help"))
    {
        std::exit(1);
    }

    std::string cascade = rf.findFileByName(xmlCascade);

    if (cascade.empty() || !object_cascade.load(cascade))
    {
        CD_ERROR("No cascade!\n");
        std::exit(1);
    }

    if (cropSelector != 0)
    {
        processor.reset();
        inCropSelectorPort->setReader(processor);
    }

    PeriodicThread::setPeriod(rateMs * 0.001);
    PeriodicThread::start();
}

/************************************************************************/

void SegmentorThread::run()
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;

    if (!camera->getImage(inYarpImg))
    {
        return;
    }

    // {yarp ImageOf Rgb -> openCv Mat Bgr}
    cv::Mat inCvMat = cv::cvarrToMat((IplImage*)inYarpImg.getIplImage());
    cv::cvtColor(inCvMat, inCvMat, CV_RGB2GRAY);

    std::vector<cv::Rect> objects;

    object_cascade.detectMultiScale(inCvMat, objects, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg = inYarpImg;
    yarp::sig::PixelRgb red(255, 0, 0);
    yarp::sig::PixelRgb green(0, 255, 0);

    yarp::os::Bottle output;

    int closestObject = 999999;
    int minimumDistance = 999999;

    for (int i = 0; i < objects.size(); i++)
    {
        const int pxX = objects[i].x + objects[i].width / 2;
        const int pxY = objects[i].y + objects[i].height / 2;

        int centerX = inCvMat.rows / 2;
        int centerY = inCvMat.cols / 2;

        int distance = std::sqrt(std::pow(pxX - centerX, 2) + std::pow(pxY - centerY, 2));

        if (distance < minimumDistance)
        {
            minimumDistance = distance;
            closestObject = i;
        }
    }

    for (int i = 0; i < objects.size(); i++)
    {
        const int pxX = objects[i].x + objects[i].width / 2;
        const int pxY = objects[i].y + objects[i].height / 2;

        if (i == closestObject)
        {
            yarp::sig::draw::addRectangleOutline(outYarpImg, green, pxX, pxY,
                    objects[i].width / 2, objects[i].height / 2);

            // scale centroids and fit into [-1, 1] range
            double cX = 2.0 * pxX / inCvMat.rows - 1.0;
            double cY = 2.0 * pxY / inCvMat.cols - 1.0;

            output.addDouble(cX); // Points right
            output.addDouble(cY); // Points down
        }
        else
        {
            yarp::sig::draw::addRectangleOutline(outYarpImg, red, pxX, pxY,
                    objects[i].width / 2, objects[i].height / 2);
        }
    }

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

    if (output.size() > 0)
    {
        pOutPort->write(output);
    }
}
