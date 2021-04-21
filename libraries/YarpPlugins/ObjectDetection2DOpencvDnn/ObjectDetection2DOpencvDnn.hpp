// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTOR_HPP__
#define __COLOR_REGION_DETECTOR_HPP__

#include <yarp/os/Searchable.h>

#include <yarp/dev/DeviceDriver.h>

#include <opencv2/dnn.hpp>


#include "IDetector.hpp"

using namespace cv::dnn;
namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup ObjectDetection2DOpencvDnn
 * @brief Contains roboticslab::ObjectDetection2DOpencvDnn.
 */
class ObjectDetection2DOpencvDnn :  public yarp::dev::DeviceDriver, public IDetector
{
public:
    bool open(yarp::os::Searchable& config) override;

    bool detect(const yarp::sig::Image& inYarpImg,
                std::vector<yarp::os::Property>& detectedObjects) override;

private:
    float confThreshold; // Confidence threshold
    float nmsThreshold; // Non-max supression threshold
    std::string modelFile; 
    std::string configDNNFile;
    std::string framework;
    std::string classesFile;
    uint backend;
    uint target; 
    Net net;
    std::vector<std::string> classes;
    std::vector<std::string> outNames;

    float scale;
    cv::Scalar mean;
    bool swapRB;
    int inpWidth;
    int inpHeight;

    static const std::string DEFAULT_MODEL_FILE;
    static const std::string DEFAULT_CONFIG_DNN_FILE;
    static const std::string DEFAULT_FRAMEWORK;
    static const std::string DEFAULT_CLASSES_FILE;
    static const uint        DEFAULT_BACKEND;
    static const uint        DEFAULT_TARGET;
    static const double      DEFAULT_SCALE;
    static const uint        DEFAULT_WIDTH;
    static const uint        DEFAULT_HEIGHT;
    static const bool        DEFAULT_RGB;
    static const double      DEFAULT_MEAN;
    static const double      DEFAULT_CONF_THR;
    static const double      DEFAULT_NMS_THR;

    inline void preprocess(const cv::Mat& frame, Net& net, cv::Size inpSize, float scale, const cv::Scalar& mean, bool swapRB);


};

}  // namespace roboticslab

#endif  // __COLOR_REGION_DETECTOR_HPP__ __OBJECT_DETECTION_2D_OPENCV_DNN__
