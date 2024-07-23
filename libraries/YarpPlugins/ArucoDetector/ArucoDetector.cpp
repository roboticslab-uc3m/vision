// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ArucoDetector.hpp"

#include <vector>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/cv/Cv.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(AC, "rl.ArucoDetector")
}

constexpr auto DEFAULT_ARUCO_SIZE = "ARUCO_ORIGINAL";

bool ArucoDetector::open(yarp::os::Searchable& config)
{
    // default params
    detectorParams = cv::aruco::DetectorParameters();

    // dictionary depends on aruco size
    std::string aruco_size = config.check("aruco_size", yarp::os::Value(DEFAULT_ARUCO_SIZE)).asString();

    if (aruco_size == "4X4_1000")
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    else if (aruco_size == "ARUCO_ORIGINAL")
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    else if (aruco_size == "6X6_1000")
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    else if (aruco_size == "7X7_1000")
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
    else
    {
        yCError(AC) << "unknown aruco_size";
        return false;
    }

    return true;
}

bool ArucoDetector::detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;

    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(inCvMat, corners, markerIds, rejectedCandidates);

    for (auto i = 0; i < corners.size(); i++)
    {
        const auto & tl = corners[i][0];
        const auto & tr = corners[i][1];
        const auto & br = corners[i][2];
        const auto & bl = corners[i][3];

        detectedObjects.addDict() = {
            {"tlx", yarp::os::Value(tl.x)},
            {"tly", yarp::os::Value(tl.y)},
            {"trx", yarp::os::Value(tr.x)},
            {"try", yarp::os::Value(tr.y)},
            {"brx", yarp::os::Value(br.x)},
            {"bry", yarp::os::Value(br.y)},
            {"blx", yarp::os::Value(bl.x)},
            {"bly", yarp::os::Value(bl.y)},
            {"text", yarp::os::Value(markerIds[i])}
        };
    }

    return true;
}
