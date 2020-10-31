// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetector.hpp"

#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>

#include <ColorDebug.h>

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
        CD_DEBUG("\"xmlCascade\" parameter found: \"%s\"\n", xmlCascade.c_str());
    }
    CD_DEBUG("Using \"xmlCascade\":\n", xmlCascade.c_str());

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("HaarDetector"); //rf.setDefaultContext(context);
    std::string xmlCascadeFullName = rf.findFileByName(xmlCascade);
    if(xmlCascadeFullName.empty())
    {
        CD_ERROR("xmlCascadeFullName NOT found\n");
        return false;
    }
    CD_DEBUG("xmlCascadeFullName \"%s\" found\n", xmlCascadeFullName.c_str());

    if (!object_cascade.load(xmlCascadeFullName))
    {
        CD_ERROR("Cannot load xmlCascadeFullName!\n");
        return false;
    }
    CD_SUCCESS("\n");

    return true;
}

/*****************************************************************/

bool HaarDetector::detect(const yarp::sig::FlexImage &inYarpImg,
                          std::vector<yarp::os::Property> &detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImgRgb;
    //inYarpImgRgb.setExternal(inYarpImg.getRawImage(), inYarpImg.width(), inYarpImg.height());
    inYarpImgRgb.copy(inYarpImg);

    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgRgb);

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
