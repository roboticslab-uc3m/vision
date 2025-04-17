// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTOR_HPP__
#define __COLOR_REGION_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>

#include "IDetector.hpp"
#include "ColorRegionDetector_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup ColorRegionDetector
 * @brief Contains ColorRegionDetector.
 */
class ColorRegionDetector : public yarp::dev::DeviceDriver,
                            public roboticslab::IDetector,
                            public ColorRegionDetector_ParamsParser
{
public:
    bool open(yarp::os::Searchable& config) override;
    bool detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects) override;
};

#endif // __COLOR_REGION_DETECTOR_HPP__
