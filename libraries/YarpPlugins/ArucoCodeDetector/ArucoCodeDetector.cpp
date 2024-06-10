// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ArucoCodeDetector.hpp"

#include <vector>

using namespace roboticslab;

bool ArucoCodeDetector::open(yarp::os::Searchable& config) 
{
    // default params
    detectorParams = cv::aruco::DetectorParameters();
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}

bool ArucoCodeDetector::detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects) 
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;

    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(inputImage, corners, markerIds, rejectedCandidates);

    for (auto i = 0; i < corners.size() / 4; i++)
    {
        const auto & tl = corners[4 * i];
        const auto & tr = corners[4 * i + 1];
        const auto & br = corners[4 * i + 2];
        const auto & bl = corners[4 * i + 3];

        detectedObjects.addDict() = {
            {"tlx", yarp::os::Value(tl.x)},
            {"tly", yarp::os::Value(tl.y)},
            {"trx", yarp::os::Value(tr.x)},
            {"try", yarp::os::Value(tr.y)},
            {"brx", yarp::os::Value(br.x)},
            {"bry", yarp::os::Value(br.y)},
            {"blx", yarp::os::Value(bl.x)},
            {"bly", yarp::os::Value(bl.y)}
        };
    }

    return true;
}
