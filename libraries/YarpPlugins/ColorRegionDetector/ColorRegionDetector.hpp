// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTOR_HPP__
#define __COLOR_REGION_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup ColorRegionDetector
 * @brief Contains roboticslab::ColorRegionDetector.
 */
class ColorRegionDetector : public yarp::dev::DeviceDriver,
                            public IDetector
{
public:
    bool open(yarp::os::Searchable& config) override;
    bool detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects) override;

private:
    std::string algorithm;
    double morphClosing;
    int threshold;
    int maxNumBlobs;
};

} // namespace roboticslab

#endif // __COLOR_REGION_DETECTOR_HPP__
