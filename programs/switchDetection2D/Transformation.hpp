// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRANSFORMATION_HPP__
#define __TRANSFORMATION_HPP__

#include <vector>

#include <yarp/os/all.h>

//#include "ColorRegionDetection2D.hpp"
//#include "HaarDetection2D.hpp"
//#include "TensorflowDetection2D.hpp"
//#include "TensorflowDetector.hpp"
//#include "TravisLib.hpp"

#include "ColorDebug.h"

namespace roboticslab
{

class Transformation
{
public:
    Transformation() : valid(false) {}
    virtual ~Transformation() {}
    bool isValid() const { return valid; }
    virtual double transform(double value) = 0;
protected:
    bool valid;
};

}  // namespace roboticslab

#endif // __TRANSFORMATION_HPP__
