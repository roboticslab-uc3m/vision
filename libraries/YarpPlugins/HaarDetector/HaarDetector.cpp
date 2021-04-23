// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetector.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>

namespace roboticslab
{

/*****************************************************************/

const std::string HaarDetector::DEFAULT_XMLCASCADE = "haarcascade_frontalface_alt.xml";

/*****************************************************************/

bool HaarDetector::open(yarp::os::Searchable& parameters)
{
    std::string xmlCascade = DEFAULT_XMLCASCADE;
    if(parameters.check("xmlCascade"))
    {
        xmlCascade = parameters.find("xmlCascade").asString();
        yDebug() << "xmlCascade parameter found:" << xmlCascade;
    }
    yDebug() << "Using xmlCascade:" << xmlCascade;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("HaarDetector"); //rf.setDefaultContext(context);
    std::string xmlCascadeFullName = rf.findFileByName(xmlCascade);
    if(xmlCascadeFullName.empty())
    {
        yError() << "xmlCascadeFullName NOT found";
        return false;
    }
    yDebug() << "xmlCascadeFullName found:" << xmlCascadeFullName;

    if (!object_cascade.load(xmlCascadeFullName))
    {
        yError() << "Cannot load xmlCascadeFullName!";
        return false;
    }

    return true;
}

/*****************************************************************/

bool HaarDetector::detect(const yarp::sig::Image &inYarpImg,
                          std::vector<yarp::os::Property> &detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    std::vector<cv::Rect> objects;
    object_cascade.detectMultiScale(inCvMat, objects, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    for(size_t i; i<objects.size(); i++)
    {
        yarp::os::Property detectedObject;
        detectedObject.put("tlx", objects[i].x);
        detectedObject.put("tly", objects[i].y);
        detectedObject.put("brx", objects[i].x + objects[i].width);
        detectedObject.put("bry", objects[i].y + objects[i].height);
        detectedObjects.push_back(detectedObject);
    }

    return true;
}

/************************************************************************/

}  // namespace roboticslab
