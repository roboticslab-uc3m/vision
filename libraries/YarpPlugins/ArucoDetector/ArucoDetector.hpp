// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ARUCO_DETECTOR_HPP__
#define __ARUCO_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>

#include <opencv2/objdetect/aruco_detector.hpp>

#include "IDetector.hpp"
#include "ArucoDetector_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup ArucoDetector
 * @brief Contains ArucoDetector.
 */

class ArucoDetector : public yarp::dev::DeviceDriver,
                      public roboticslab::IDetector,
                      public ArucoDetector_ParamsParser
{
public:
    bool open(yarp::os::Searchable& config) override;
    bool detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects) override;

private:
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::Dictionary dictionary;
};

#endif // __ARUCO_DETECTOR_HPP__
