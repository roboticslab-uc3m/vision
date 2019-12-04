// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRANSFORMATION_HPP__
#define __TRANSFORMATION_HPP__

#include <vector>

#include <yarp/os/all.h>

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

class HaarDetectionTransformation : public Transformation
{
public:
    HaarDetectionTransformation(yarp::os::Searchable* parameters);
    double transform(double value) override;
private:
    double m, b;

};

class ColorRegionDetectionTransformation : public Transformation
{
public:
    ColorRegionDetectionTransformation(yarp::os::Searchable* parameters);
    double transform(double value) override;
private:
    double m, b;
};

class TensorflowDetectionTransformation : public Transformation
{
public:
    TensorflowDetectionTransformation(yarp::os::Searchable* parameters);
    double transform(double value) override;
private:
    double m, b;

};

}  // namespace roboticslab

#endif // __TRANSFORMATION_HPP__
