// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTOR_HPP__
#define __HAAR_DETECTOR_HPP__

#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/sig/Image.h>

#include <opencv2/objdetect/objdetect.hpp>

#include <ColorDebug.h>

#include "Detector.hpp"

#define DEFAULT_XMLCASCADE "haarcascade_cocacola_can.xml"

namespace roboticslab
{

class HaarDetector : public Detector
{
public:
    HaarDetector(yarp::os::Searchable* parameters);
    yarp::sig::ImageOf<yarp::sig::PixelRgb> detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg) override;
private:
    cv::CascadeClassifier object_cascade;
    cv::Mat inCvMatPost;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImgPre;
};

}  // namespace roboticslab

#endif  // __HAAR_DETECTOR_HPP__
