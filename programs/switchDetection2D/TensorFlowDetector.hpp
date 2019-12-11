// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TENSOR_FLOW_DETECTOR_HPP__
#define __TENSOR_FLOW_DETECTOR_HPP__

#include <yarp/os/Searchable.h>

#include <yarp/dev/FrameGrabberInterfaces.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tensorflow/cc/ops/const_op.h>
#include <tensorflow/cc/ops/image_ops.h>
#include <tensorflow/cc/ops/standard_ops.h>
#include <tensorflow/core/framework/graph.pb.h>
#include <tensorflow/core/graph/default_device.h>
#include <tensorflow/core/graph/graph_def_builder.h>
#include <tensorflow/core/lib/core/threadpool.h>
#include <tensorflow/core/lib/io/path.h>
#include <tensorflow/core/lib/strings/stringprintf.h>
#include <tensorflow/core/platform/init_main.h>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/util/command_line_flags.h>

#include "TensorFlowDetector.hpp"

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

    tensorflow::string inputLayer;
    tensorflow::TensorShape shape;
    std::vector<tensorflow::Tensor> outputs;
    std::unique_ptr<tensorflow::Session> session;
    std::map<int, std::string> labelsMap;
    std::vector<std::string> outputLayer;
    double thresholdScore = 0.5;
    double thresholdIOU = 0.8;

    static const std::string DEFAULT_TRAINEDMODEL;
    static const std::string DEFAULT_TRAINEDMODEL_LABELS;
};

}  // namespace roboticslab

#endif  // __TENSOR_FLOW_DETECTOR_HPP__
