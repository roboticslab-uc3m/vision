// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TENSOR_FLOW_DETECTOR_HPP__
#define __TENSOR_FLOW_DETECTOR_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/FrameGrabberInterfaces.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"

#include "TensorFlowDetector.hpp"

#include "Detector.hpp"

#include <ColorDebug.h>

namespace roboticslab
{

class TensorFlowDetector : public Detector
{
public:
    TensorFlowDetector(yarp::os::Searchable* parameters);
    void configuration(std::string trainedModel, std::string trainedModelLabels, yarp::sig::ImageOf<yarp::sig::PixelRgb> *inYarpImg);
    bool detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                std::vector<DetectedObject*>& detectedObjects,
                yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret);

private:
    // Tensorflow: Session object instance
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;
    tensorflow::Status readLabelsMapStatus;
    tensorflow::Status loadGraphStatus;
    tensorflow::string inputLayer;
    tensorflow::TensorShape shape;
    tensorflow::Tensor tensor;
    std::vector<tensorflow::Tensor> outputs;
    std::unique_ptr<tensorflow::Session> session;
    std::map<int, std::string> labelsMap;
    std::vector<std::string> outputLayer;
    time_t start, end;
    int nFrames = 25;
    int iFrame = 0;
    double fps = 0.;
    double thresholdScore = 0.5;
    double thresholdIOU = 0.8;
    int initDetector=0;
    std::string trainedModelFullName;
    std::string trainedModelLabelsFullName;

    yarp::os::Bottle bottle;

    static const std::string DEFAULT_TRAINEDMODEL;
    static const std::string DEFAULT_TRAINEDMODEL_LABELS;
};

}  // namespace roboticslab

#endif  // __TENSOR_FLOW_DETECTOR_HPP__
