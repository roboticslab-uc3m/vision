// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __OPENCV_DNN_DETECTOR_HPP__
#define __OPENCV_DNN_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>

#include <opencv2/dnn.hpp>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup OpencvDnnDetector
 * @brief Contains roboticslab::OpencvDnnDetector.
 */
class OpencvDnnDetector : public yarp::dev::DeviceDriver,
                          public IDetector
{
public:
    bool open(yarp::os::Searchable& config) override;

    bool detect(const yarp::sig::Image& inYarpImg, std::vector<yarp::os::Property>& detectedObjects) override;

private:
    cv::dnn::Net net;
    std::vector<std::string> classes;
    std::vector<std::string> outNames;

    float confThreshold; // Confidence threshold
    float nmsThreshold; // Non-max supression threshold
    float scale;
    cv::Scalar mean;

    void preprocess(const cv::Mat& frame);
    void postprocess(const cv::Size& size, const std::vector<cv::Mat>& outs, std::vector<yarp::os::Property> &detectedObjects);
};

} // namespace roboticslab

#endif // __OPENCV_DNN_DETECTOR_HPP__
