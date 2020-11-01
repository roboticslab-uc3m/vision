// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DETECTOR_HPP__
#define __DETECTOR_HPP__

#include <vector>

#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

namespace roboticslab
{

class IDetector
{
public:
    virtual ~IDetector() {}
    virtual bool detect(const yarp::sig::Image& inYarpImg,
                        std::vector<yarp::os::Property>& detectedObjects) = 0;
};

}  // namespace roboticslab

#endif // __DETECTOR_HPP__
