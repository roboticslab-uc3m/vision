// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ARUCO_DETECTOR_HPP__
#define __ARUCO_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup ArucoCodeDetector
 * @brief Contains roboticslab::ArucoCodeDetector.
 */

class ArucoCodeDetector : public yarp::dev::DeviceDriver,
                          public IDetector
{
public:
    bool open(yarp::os::Searchable& config) override;
    bool detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects) override;

private:
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::Dictionary dictionary;
};

}

#endif // __ARUCO_DETECTOR_HPP__
