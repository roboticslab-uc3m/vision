// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetector.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/cv/Cv.h>

#ifdef HAVE_CV_FACE
# include <opencv2/face/facemarkLBF.hpp>
#endif

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(HAAR, "rl.HaarDetector")
}

bool HaarDetector::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(HAAR) << "Failed to parse parameters";
        return false;
    }

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("HaarDetector");

    std::string xmlCascadeFullName = rf.findFileByName(m_xmlCascade);

    if (xmlCascadeFullName.empty())
    {
        yCError(HAAR) << "xmlCascadeFullName not found";
        return false;
    }

    yCDebug(HAAR) << "xmlCascadeFullName found:" << xmlCascadeFullName;

    if (!object_cascade.load(xmlCascadeFullName))
    {
        yCError(HAAR) << "Cannot load xmlCascadeFullName";
        return false;
    }

    if (!m_lbfModel.empty())
    {
#ifdef HAVE_CV_FACE
        std::string lfbModelFullName = rf.findFileByName(m_lbfModel);

        if (lfbModelFullName.empty())
        {
            yCError(HAAR) << "LBF face landmark model not found";
            return false;
        }

        facemark = cv::face::FacemarkLBF::create();
        facemark->loadModel(lfbModelFullName);
        yCDebug(HAAR) << "Loaded face landmark model:" << lfbModelFullName;
#else
        yCError(HAAR) << "LBF face landmark model specified, but OpenCV was not built with face module";
        return false;
#endif
    }

    return true;
}

yarp::dev::ReturnValue HaarDetector::detect(const yarp::sig::Image & inYarpImg, yarp::os::Bottle & detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    std::vector<cv::Rect> objects;
    object_cascade.detectMultiScale(inCvMat, objects, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    for (const auto & object : objects)
    {
        yarp::os::Property dict = {
            {"tlx", yarp::os::Value(object.x)},
            {"tly", yarp::os::Value(object.y)},
            {"brx", yarp::os::Value(object.x + object.width)},
            {"bry", yarp::os::Value(object.y + object.height)}
        };

        detectedObjects.addList().fromString(dict.toString());
    }

#ifdef HAVE_CV_FACE
    if (facemark)
    {
        std::vector<std::vector<cv::Point2f>> shapes;

        if (facemark->fit(inCvMat, objects, shapes))
        {
            for (auto i = 0; i < objects.size(); i++)
            {
                auto * dict = detectedObjects.get(i).asList();

                auto & landmarks = dict->addList();
                landmarks.addString("landmarks");

                auto & landmarkList = landmarks.addList();

                for (const auto & shape : shapes[i])
                {
                    landmarkList.addList() = {
                        yarp::os::Value(shape.x, false),
                        yarp::os::Value(shape.y, false)
                    };
                }
            }
        }
    }
#endif

    return yarp::dev::ReturnValue_ok;
}
