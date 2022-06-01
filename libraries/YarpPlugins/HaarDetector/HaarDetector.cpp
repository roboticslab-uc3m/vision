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
    auto xmlCascade = parameters.check("xmlCascade", yarp::os::Value(DEFAULT_XMLCASCADE)).asString();
    yCDebug(HAAR) << "Using xmlCascade:" << xmlCascade;

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("HaarDetector");

    if (parameters.check("useKazemi", "enable Kazemi detector"))
    {
        facemark = cv::face::FacemarkLBF::create();
        facemark->loadModel("/home/carlos/HogFaceDetector/vision/models/lbfmodel/lbfmodel.yaml");

//        facemark = cv::face::FacemarkKazemi::create();
//        facemark->loadModel("/home/carlos/HogFaceDetector/vision/face_landmark_model.dat");

        printf("Loaded model\n");
    }

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

    yarp::os::Property & dict = detectedObjects.addDict();

    for (const auto & object : objects)
    {
        dict.put("tlx", object.x);
        dict.put("tly", object.y);
        dict.put("brx", object.x + object.width);
        dict.put("bry", object.y + object.height);
    }

    yarp::os::Value * list = yarp::os::Value::makeList();

    if (facemark)
    {
        facemark->fit(inCvMat, objects, shapes);

        for (unsigned long i = 0; i < objects.size(); i++)
        {
            for(unsigned long k = 0; k < shapes[i].size(); k++)
            {
                list->asList()->addList() = {
                        yarp::os::Value(shapes[i][k].x, false),
                        yarp::os::Value(shapes[i][k].y, false)
                };
            }
        }

        dict.put("landmarks", list);
    }

    return true;
}
