// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DNN_DETECTOR_HPP__
#define __DNN_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>

#include <opencv2/dnn.hpp>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup DnnDetector
 * @brief Contains roboticslab::DnnDetector.
 */
class DnnDetector : public yarp::dev::DeviceDriver,
                    public IDetector
{
public:
    bool open(yarp::os::Searchable & config) override;
    bool detect(const yarp::sig::Image & inYarpImg, yarp::os::Bottle & detectedObjects) override;

private:
    cv::dnn::Net net;
    std::vector<std::string> classes;
    std::vector<std::string> outNames;

    float confThreshold; // Confidence threshold
    float nmsThreshold; // Non-max supression threshold
    float scale;
    cv::Scalar mean;

    void preprocess(const cv::Mat & frame);
    void postprocess(const cv::Size & size, const std::vector<cv::Mat> & outs, yarp::os::Bottle & detectedObjects);
};

} // namespace roboticslab

#endif // __DNN_DETECTOR_HPP__
