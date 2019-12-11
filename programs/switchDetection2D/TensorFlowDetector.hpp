// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TENSOR_FLOW_DETECTOR_HPP__
#define __TENSOR_FLOW_DETECTOR_HPP__

#include <yarp/os/Searchable.h>

#include <yarp/dev/FrameGrabberInterfaces.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tensorflow/core/public/session.h>

#include "Detector.hpp"

namespace roboticslab
{

class TensorFlowDetector : public Detector
{
public:
    TensorFlowDetector(yarp::os::Searchable* parameters);
    bool detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                std::vector<DetectedObject*>& detectedObjects,
                yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret);

private:
    tensorflow::string inputLayer;
    tensorflow::TensorShape shape;
    std::vector<tensorflow::Tensor> outputs;
    std::unique_ptr<tensorflow::Session> session;
    std::map<int, std::string> labelsMap;
    std::vector<std::string> outputLayer;

    void setTensorShape(tensorflow::int64 h, tensorflow::int64 w);
    bool firstArrived;

    tensorflow::Status loadGraph(const tensorflow::string &graph_file_name,
                                 std::unique_ptr<tensorflow::Session> *session);
    tensorflow::Status readLabelsMapFile(const tensorflow::string &fileName, std::map<int, tensorflow::string> &labelsMap);
    tensorflow::Status readTensorFromMat(const cv::Mat &mat, tensorflow::Tensor &outTensor);
    void drawBoundingBoxOnImage(cv::Mat &image, double xMin, double yMin, double xMax, double yMax,
                                double score, std::string label, bool scaled);
    void drawBoundingBoxesOnImage(cv::Mat &image,
                                  tensorflow::TTypes<float>::Flat &scores,
                                  tensorflow::TTypes<float>::Flat &classes,
                                  tensorflow::TTypes<float,3>::Tensor &boxes,
                                  std::map<int, tensorflow::string> &labelsMap,
                                  std::vector<size_t> &idxs);
    double IOU(cv::Rect2f box1, cv::Rect2f box2);
    std::vector<size_t> filterBoxes(tensorflow::TTypes<float>::Flat &scores,
                                    tensorflow::TTypes<float, 3>::Tensor &boxes,
                                    double thresholdIOU, double thresholdScore);

    static const std::string DEFAULT_TRAINEDMODEL;
    static const std::string DEFAULT_TRAINEDMODEL_LABELS;
    static const double DEFAULT_THRESHOLD_SCORE;
    static const double DEFAULT_THRESHOLD_IOU;
};

}  // namespace roboticslab

#endif  // __TENSOR_FLOW_DETECTOR_HPP__
