// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTOR_HPP__
#define __COLOR_REGION_DETECTOR_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Image.h>

#include <ColorDebug.h>

#include "TravisLib.hpp"

#include "Detector.hpp"

#define DEFAULT_ALGORITHM "blueMinusRed"
#define DEFAULT_MORPH_CLOSING 2
#define DEFAULT_THRESHOLD 55

#define DEFAULT_MAX_NUM_BLOBS 1

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

};

}  // namespace roboticslab

#endif  // __COLOR_REGION_DETECTOR_HPP__
