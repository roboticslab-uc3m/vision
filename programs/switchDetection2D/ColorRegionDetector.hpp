// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTOR_HPP__
#define __COLOR_REGION_DETECTOR_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Image.h>

#include <ColorDebug.h>

#include "TravisLib.hpp"

#include "Detector.hpp"

namespace roboticslab
{

class ColorRegionDetector : public Detector
{
public:
    ColorRegionDetector(yarp::os::Searchable* parameters);
    bool detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                std::vector<DetectedObject*>& detectedObjects,
                yarp::sig::ImageOf<yarp::sig::PixelRgb> &ret) override;

private:
    std::string algorithm;
    double morphClosing;
    int threshold;

    int maxNumBlobs;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outImageProcessed;
    yarp::os::Bottle outputProcessed;

    static const std::string DEFAULT_ALGORITHM;
    static const double DEFAULT_MORPH_CLOSING;
    static const double DEFAULT_THRESHOLD;

    static const double DEFAULT_MAX_NUM_BLOBS;
};

}  // namespace roboticslab

#endif  // __COLOR_REGION_DETECTOR_HPP__
