// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <time.h>
#include <utility>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

#include <cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

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

#include <ColorDebug.h>

#include "DetectorThread.hpp"
#include "TensorFlowDetectorHelper.hpp"
#include "TensorFlowDetector.hpp"

namespace roboticslab
{

/*****************************************************************/

const std::string TensorFlowDetector::DEFAULT_TRAINEDMODEL = "frozen_inference_graph.pb";
const std::string TensorFlowDetector::DEFAULT_TRAINEDMODEL_LABELS = "labels_map.pbtxt";

/*****************************************************************/

TensorFlowDetector::TensorFlowDetector(yarp::os::Searchable* parameters)
{
    std::string trainedModel = DEFAULT_TRAINEDMODEL;
    CD_DEBUG("*** --trainedModel [file.pb] (default: \"%s\")\n", trainedModel.c_str());
    if(parameters->check("trainedModel"))
    {
        CD_INFO("**** \"trainedModel\" parameter for TensorFlowDetector found\n");
        trainedModel = parameters->find("trainedModel").asString();
    }
    CD_DEBUG("**** \"trainedModel\" parameter for TensorFlowDetector found: \"%s\"\n", trainedModel.c_str());

    std::string trainedModelLabels = DEFAULT_TRAINEDMODEL_LABELS;
    printf("\t--trainedModelLabels [file.pbtxt] (default: \"%s\")\n", trainedModelLabels.c_str());
    if(parameters->check("trainedModelLabels"))
    {
        CD_INFO("**** \"trainedModelLabels\" parameter for TensorFlowDetector found\n");
        trainedModelLabels = parameters->find("trainedModelLabels").asString();
    }
    CD_DEBUG("**** \"trainedModelLabels\" parameter for TensorFlowDetector found: \"%s\"\n", trainedModelLabels.c_str());

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("switchDetection2D");

    std::string trainedModelFullName = rf.findFileByName(trainedModel);
    if(trainedModelFullName.empty())
    {
        CD_ERROR("**** full path for trainedModel NOT found\n");
        return;
    }
    CD_DEBUG("**** full path for trainedModel found: \"%s\"\n", trainedModelFullName.c_str());

    std::string trainedModelLabelsFullName = rf.findFileByName(trainedModelLabels);
    if(trainedModelLabelsFullName.empty())
    {
        CD_ERROR("**** full path for trainedModelLabels NOT found\n");
        return;
    }
    CD_DEBUG("**** full path for trainedModelLabels found: \"%s\"\n", trainedModelLabelsFullName.c_str());

    //outPortShape.open("/tensorflowDetection2D/shape");
    //yarp::sig::ImageOf<yarp::sig::PixelRgb> *inYarpImg=outPortShape.read();;
    //j//tensorflowDetector.configuration(model, labels, inYarpImg);

}

/*****************************************************************/

void TensorFlowDetector::configuration(std::string trainedModel, std::string trainedModelLabels, yarp::sig::ImageOf<yarp::sig::PixelRgb> *inYarpImg/*, yarp::os::BufferedPort<ImageOf<PixelRgb> > inputPort*/){


    //yarp::sig::ImageOf<yarp::sig::PixelRgb> *inYarpImgShape = inputPort.read();

    //yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg=&inYarpImg;
    yarp::os::Bottle output;

    std::string LABELS = trainedModelLabels;
    std::string GRAPH = trainedModel;

    // Set  node names
    inputLayer = "image_tensor:0";
    outputLayer = {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"};

    // Load .pb frozen model
    tensorflow::string graphPath = GRAPH;
    loadGraphStatus = loadGraph(graphPath, &session);
    if (!loadGraphStatus.ok())
    {
        std::cout<<"Fail loading graph "<<graphPath<<"."<<std::endl;
    }
    else
        std::cout<<"Graph "<<graphPath<<" loaded correctly."<<std::endl;


    // Load labels
    labelsMap = std::map<int,std::string>();
    std::cout<<"Labels "<<LABELS<<" are going to be loaded."<<std::endl;
    readLabelsMapStatus = readLabelsMapFile(LABELS, labelsMap);
    if (!readLabelsMapStatus.ok())
    {
        std::cout<<"Fail loading labels "<<LABELS<<"."<<std::endl;
    } else
        std::cout<<"Labels "<<LABELS<<" loaded correctly."<<std::endl;
        std::cout<<labelsMap.size()<<" labels have been loaded."<<std::endl;

    time(&start);
    shape = tensorflow::TensorShape();
    shape.AddDim(1);
    shape.AddDim((tensorflow::int64)inYarpImg->height());
    shape.AddDim((tensorflow::int64)inYarpImg->width());
    shape.AddDim(3);
    std::cout<<"Taking frames..."<<std::endl;

    valid = true;
}

/*****************************************************************/

bool TensorFlowDetector::detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                                std::vector<DetectedObject*>& detectedObjects,
                                yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret)
{
    cv::Mat inCvMat = cv::cvarrToMat((IplImage*)inYarpImg.getIplImage());
    cv::cvtColor(inCvMat, inCvMat, cv::COLOR_BGR2RGB);

    std::cout << "Frame: " << iFrame << std::endl;

    if (nFrames % (iFrame + 1) == 0)
    {
        time(&end);
        fps = 1. * nFrames / difftime(end, start);
        time(&start);
    }
    iFrame++;

    // Mat -> Tensor
    tensor = tensorflow::Tensor(tensorflow::DT_FLOAT, shape);
    tensorflow::Status readTensorStatus = readTensorFromMat(inCvMat, tensor);
    if (!readTensorStatus.ok())
    {
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<"Mat OpenCV -> Tensor : FAIL"<<std::endl;
    }

    // Execute graph
    outputs.clear();
    tensorflow::Status runStatus = session->Run({{inputLayer, tensor}}, outputLayer, {}, &outputs);
    if (!runStatus.ok())
    {
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<"Running model status: FAIL"<<std::endl;
    }

    // Extract results
    tensorflow::TTypes<float>::Flat scores = outputs[1].flat<float>();
    tensorflow::TTypes<float>::Flat classes = outputs[2].flat<float>();
    tensorflow::TTypes<float>::Flat numDetections = outputs[3].flat<float>();
    tensorflow::TTypes<float, 3>::Tensor boxes = outputs[0].flat_outer_dims<float,3>();

    std::vector<size_t> goodIdxs = filterBoxes(scores, boxes, thresholdIOU, thresholdScore);
    for (size_t i = 0; i < goodIdxs.size(); i++)
    {
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<"Detection: "<<labelsMap[classes(goodIdxs.at(i))]<< " -> Score: "<<scores(goodIdxs.at(i))<<std::endl;


        double score_detection=scores(goodIdxs.at(i));
        std::string class_name=std::string(labelsMap[classes(goodIdxs.at(i))]);
        bottle.clear();
        bottle.addString(" Detection number: ");
        bottle.addInt(goodIdxs.size());
        bottle.addString(" Detection: ");
        bottle.addString(class_name);
        bottle.addString(" Score: ");
        bottle.addDouble(score_detection);

        std::cout<<std::endl;
        drawBoundingBoxesOnImage(inCvMat, scores, classes, boxes, labelsMap, goodIdxs);
        cv::putText(inCvMat, std::to_string(fps).substr(0, 5), cv::Point(0, inCvMat.rows), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255));
        cv::cvtColor(inCvMat, inCvMat, cv::COLOR_BGR2RGB);

    }

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg = inYarpImg;
    outYarpImg.setExternal(inCvMat.data,inCvMat.size[1],inCvMat.size[0]);

    outYarpImg = ret;
    return true;
}

// -----------------------------------------------------------------------------

}// namespace roboticslab
