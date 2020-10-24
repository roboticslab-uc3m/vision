// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTOR_HPP__
#define __COLOR_REGION_DETECTOR_HPP__

#include <yarp/os/Searchable.h>

#include <yarp/dev/DeviceDriver.h>

#include "IDetector.hpp"

namespace roboticslab
{

class ColorRegionDetector :  public yarp::dev::DeviceDriver, public IDetector
{
public:
    bool open(yarp::os::Searchable& config) override;

    bool detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                yarp::sig::VectorOf<DetectedObject>& detectedObjects) override;

private:
    std::string algorithm;
    double morphClosing;
    int threshold;

    int maxNumBlobs;

    static const std::string DEFAULT_ALGORITHM;
    static const double DEFAULT_MORPH_CLOSING;
    static const double DEFAULT_THRESHOLD;

    static const double DEFAULT_MAX_NUM_BLOBS;
};

}  // namespace roboticslab

#endif  // __COLOR_REGION_DETECTOR_HPP__
