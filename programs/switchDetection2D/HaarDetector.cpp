// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetector.hpp"

#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/os/ResourceFinder.h>

#include <yarp/sig/ImageDraw.h>

namespace roboticslab
{

HaarDetector::HaarDetector(yarp::os::Searchable* parameters)
{
    std::string xmlCascade = DEFAULT_XMLCASCADE;
    CD_DEBUG("*** \"xmlCascade\" [file.xml] (default: \"%s\")\n", xmlCascade.c_str());
    if(parameters->check("xmlCascade"))
    {
        xmlCascade = parameters->find("xmlCascade").asString();
        CD_DEBUG("**** \"xmlCascade\" parameter for HaarDetectionTransformation found: \"%s\"\n", xmlCascade.c_str());
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("switchDetection2D"); //rf.setDefaultContext(context);
    std::string xmlCascadeFullName = rf.findFileByName(xmlCascade);
    if(xmlCascadeFullName.empty())
    {
        CD_ERROR("**** xmlCascadeFullName NOT found\n");
        return;
    }
    CD_DEBUG("**** xmlCascadeFullName \"%s\" found\n", xmlCascadeFullName.c_str());

    if (!object_cascade.load(xmlCascadeFullName))
    {
        CD_ERROR("Cannot load xmlCascadeFullName!\n");
        return;
    }

    CD_SUCCESS("\n");
    valid = true;
}

/*****************************************************************/
yarp::sig::ImageOf<yarp::sig::PixelRgb> HaarDetector::detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg)
{
    CD_DEBUG("\n");

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

        int centerX = inCvMat.cols / 2;
        int centerY = inCvMat.rows / 2;

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
            double cX = 2.0 * pxX / inCvMat.cols - 1.0;
            double cY = 2.0 * pxY / inCvMat.rows - 1.0;

            output.addFloat64(cX); // Points right
            output.addFloat64(cY); // Points down
        }
        else
        {
            yarp::sig::draw::addRectangleOutline(outYarpImg, red, pxX, pxY,
            objects[i].width / 2, objects[i].height / 2);
        }
    }

    return outYarpImg;

}

/************************************************************************/

}  // namespace roboticslab
