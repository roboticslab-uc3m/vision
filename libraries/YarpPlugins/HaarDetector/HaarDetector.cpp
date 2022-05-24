// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetector.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(HAAR, "rl.HaarDetector")
}

constexpr auto DEFAULT_XMLCASCADE = "haarcascade_frontalface_alt.xml";

bool HaarDetector::open(yarp::os::Searchable& parameters)
{
    if (parameters.check("useKazemi", "enable Kazemi detector"))
    {
        // load file
        //facemark = createKazemiDetector();
    }

    auto xmlCascade = parameters.check("xmlCascade", yarp::os::Value(DEFAULT_XMLCASCADE)).asString();
    yCDebug(HAAR) << "Using xmlCascade:" << xmlCascade;

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("HaarDetector");

    std::string xmlCascadeFullName = rf.findFileByName(xmlCascade);

    if (xmlCascadeFullName.empty())
    {
        yCError(HAAR) << "xmlCascadeFullName NOT found";
        return false;
    }

    yCDebug(HAAR) << "xmlCascadeFullName found:" << xmlCascadeFullName;

    if (!object_cascade.load(xmlCascadeFullName))
    {
        yCError(HAAR) << "Cannot load xmlCascadeFullName!";
        return false;
    }

    return true;
}

bool HaarDetector::detect(const yarp::sig::Image & inYarpImg, yarp::os::Bottle & detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    std::vector<cv::Rect> objects;
    object_cascade.detectMultiScale(inCvMat, objects, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    for (const auto & object : objects)
    {
        detectedObjects.addDict() = {
            {"tlx", yarp::os::Value(object.x)},
            {"tly", yarp::os::Value(object.y)},
            {"brx", yarp::os::Value(object.x + object.width)},
            {"bry", yarp::os::Value(object.y + object.height)}
        };

//        if (facemark)
//        {
//            ;
//        }
    }

    return true;
}
