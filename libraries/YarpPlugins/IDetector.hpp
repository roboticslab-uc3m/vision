// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DETECTOR_HPP__
#define __DETECTOR_HPP__

#include <vector>

#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

namespace roboticslab
{

class DetectedObject
{
public:
    void setBoundingBox(int tlx, int tly, int brx, int bry)
    {
        _tlx = tlx; _tly = tly; _brx = brx; _bry = bry;
    }
    int cx() { return (_tlx+_brx)/2; }
    int cy() { return (_tly+_bry)/2; }
    int width() { return (_brx-_tlx); }
    int height() { return (_bry-_tly); }
private:
    int _tlx, _tly, _brx, _bry;
};

class IDetector
{
public:
    virtual ~IDetector() {}
    virtual bool detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                        yarp::sig::VectorOf<DetectedObject>& detectedObjects) = 0;
};

}  // namespace roboticslab

#endif // __DETECTOR_HPP__
