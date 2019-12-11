// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTOR_HPP__
#define __HAAR_DETECTOR_HPP__

#include <yarp/os/Searchable.h>

#include <opencv2/objdetect/objdetect.hpp>

#include "Detector.hpp"

namespace roboticslab
{

class HaarDetector : public Detector
{
public:
    HaarDetector(yarp::os::Searchable* parameters);
    bool detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                std::vector<DetectedObject*>& detectedObjects,
                yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret) override;
private:
    cv::CascadeClassifier object_cascade;

    static const std::string DEFAULT_XMLCASCADE;
};

}  // namespace roboticslab

#endif  // __HAAR_DETECTOR_HPP__
