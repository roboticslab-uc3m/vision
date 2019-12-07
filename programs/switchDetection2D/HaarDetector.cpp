// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetector.hpp"

#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/os/ResourceFinder.h>

namespace roboticslab
{

/*****************************************************************/

const std::string HaarDetector::DEFAULT_XMLCASCADE = "haarcascade_frontalface_alt.xml";

/*****************************************************************/

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

bool HaarDetector::detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                          std::vector<DetectedObject*>& detectedObjects,
                          yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret)
{
    CD_DEBUG("\n");

    cv::Mat inCvMat = cv::cvarrToMat((IplImage*)inYarpImg.getIplImage());
    cv::cvtColor(inCvMat, inCvMat, CV_RGB2GRAY);

    std::vector<cv::Rect> objects;
    object_cascade.detectMultiScale(inCvMat, objects, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

    for(size_t i; i<objects.size(); i++)
    {
        DetectedObject* detectedObject = new DetectedObject;
        detectedObject->setBoundingBox(objects[i].x,
                                       objects[i].y,
                                       objects[i].x + objects[i].width,
                                       objects[i].y + objects[i].height);
        detectedObjects.push_back(detectedObject);
    }


    return true;
}

/************************************************************************/

}  // namespace roboticslab
