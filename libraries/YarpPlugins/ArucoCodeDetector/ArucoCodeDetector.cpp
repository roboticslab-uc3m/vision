// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ArucoCodeDetector.hpp"

#include <iostream>
#include <vector>
#include <yarp/os/Value.h>
#include <yarp/cv/Cv.h>

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>


using namespace roboticslab;

bool ArucoCodeDetector::open(yarp::os::Searchable& config) 
{
    // default params
    detectorParams = cv::aruco::DetectorParameters();
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    return true;
}

bool ArucoCodeDetector::detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects) 
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
        const auto & corner = corners[i];

        detectedObjects.addDict() = {
            {"tlx", yarp::os::Value(corner[0].x)},
            {"tly", yarp::os::Value(corner[0].y)},
            {"trx", yarp::os::Value(corner[1].x)},
            {"try", yarp::os::Value(corner[1].y)},
            {"brx", yarp::os::Value(corner[2].x)},
            {"bry", yarp::os::Value(corner[2].y)},
            {"blx", yarp::os::Value(corner[3].x)},
            {"bly", yarp::os::Value(corner[3].y)}
        };
    }

    return true;
}
