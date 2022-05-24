// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTOR_HPP__
#define __HAAR_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>

#include <opencv2/objdetect.hpp>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup HaarDetector
 * @brief Contains roboticslab::HaarDetector.
 */
class HaarDetector : public yarp::dev::DeviceDriver,
                     public IDetector
{
public:
    bool open(yarp::os::Searchable& config) override;
    bool detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects) override;

private:
    cv::CascadeClassifier object_cascade;
    //cv::Ptr<Facemark> facemark;
};

} // namespace roboticslab

#endif // __HAAR_DETECTOR_HPP__
