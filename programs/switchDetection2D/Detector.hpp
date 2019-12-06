// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DETECTOR_HPP__
#define __DETECTOR_HPP__

#include <vector>

#include <yarp/sig/Image.h>

#include <ColorDebug.h>

namespace roboticslab
{

class BoundingBox
{

};

class Detector
{
public:
    Detector() : valid(false) {}
    virtual ~Detector() {}
    bool isValid() const { return valid; }
    virtual bool detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                        std::vector<BoundingBox*>& boundingBoxes,
                        yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret) = 0;

protected:
    bool valid;
};

}  // namespace roboticslab

#endif // __DETECTOR_HPP__
