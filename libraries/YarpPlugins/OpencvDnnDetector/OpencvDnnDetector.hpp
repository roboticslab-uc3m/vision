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
    float confThreshold; // Confidence threshold
    float nmsThreshold; // Non-max supression threshold
    std::string modelFile;
    std::string configDNNFile;
    std::string framework;
    std::string classesFile;
    int backend;
    int target;
    cv::dnn::Net net;
    std::vector<std::string> classes;
    std::vector<std::string> outNames;

    float scale;
    cv::Scalar mean;
    bool swapRB;
    int inpWidth;
    int inpHeight;

    void preprocess(const cv::Mat& frame, cv::dnn::Net& net, cv::Size inpSize, float scale, const cv::Scalar& mean, bool swapRB);
};

} // namespace roboticslab

#endif // __OPENCV_DNN_DETECTOR_HPP__
