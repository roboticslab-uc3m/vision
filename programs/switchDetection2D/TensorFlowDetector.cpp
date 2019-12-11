// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/ResourceFinder.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

#include <cv.hpp>
#include <opencv2/core/core.hpp>
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

#include <ColorDebug.h>

#include "TensorFlowDetectorHelper.hpp"
#include "TensorFlowDetector.hpp"

namespace roboticslab
{

/*****************************************************************/

const std::string TensorFlowDetector::DEFAULT_TRAINEDMODEL = "frozen_inference_graph.pb";
const std::string TensorFlowDetector::DEFAULT_TRAINEDMODEL_LABELS = "labels_map.pbtxt";

/*****************************************************************/

TensorFlowDetector::TensorFlowDetector(yarp::os::Searchable* parameters) : firstArrived(false)
{
    std::string trainedModel = DEFAULT_TRAINEDMODEL;
    CD_DEBUG("\"trainedModel\" [file.pb] (default: \"%s\")\n", trainedModel.c_str());
    if(parameters->check("trainedModel"))
    {
        CD_INFO("\"trainedModel\" parameter for TensorFlowDetector found\n");
        trainedModel = parameters->find("trainedModel").asString();
    }
    CD_DEBUG("\"trainedModel\" parameter for TensorFlowDetector found: \"%s\"\n", trainedModel.c_str());

    std::string trainedModelLabels = DEFAULT_TRAINEDMODEL_LABELS;
    CD_DEBUG("\"trainedModelLabels\" [file.pbtxt] (default: \"%s\")\n", trainedModelLabels.c_str());
    if(parameters->check("trainedModelLabels"))
    {
        CD_INFO("\"trainedModelLabels\" parameter for TensorFlowDetector found\n");
        trainedModelLabels = parameters->find("trainedModelLabels").asString();
    }
    CD_DEBUG("\"trainedModelLabels\" parameter for TensorFlowDetector found: \"%s\"\n", trainedModelLabels.c_str());

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("switchDetection2D");

    std::string trainedModelFullName = rf.findFileByName(trainedModel);
    if(trainedModelFullName.empty())
    {
        CD_ERROR("full path for \"trainedModel\" NOT found\n");
        return;
    }
    CD_DEBUG("full path for \"trainedModel\" found: \"%s\"\n", trainedModelFullName.c_str());

    std::string trainedModelLabelsFullName = rf.findFileByName(trainedModelLabels);
    if(trainedModelLabelsFullName.empty())
    {
        CD_ERROR("full path for \"trainedModelLabels\" NOT found\n");
        return;
    }
    CD_DEBUG("full path for \"trainedModelLabels\" found: \"%s\"\n", trainedModelLabelsFullName.c_str());

    // Set  node names
    inputLayer = "image_tensor:0";
    outputLayer = {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"};

    // Load .pb frozen model
    tensorflow::string graphPath = trainedModelFullName; // GRAPH
    tensorflow::Status loadGraphStatus = loadGraph(graphPath, &session);
    if (!loadGraphStatus.ok())
    {
        CD_ERROR("Fail loading graph \"%s\"\n",graphPath.c_str());
        return;
    }
    CD_SUCCESS("Graph \"%s\" loaded correctly\n",graphPath.c_str());

    // Load labels
    labelsMap = std::map<int,std::string>();
    CD_INFO("Labels \"%s\" are going to be loaded.\n",trainedModelLabelsFullName.c_str());
    tensorflow::Status readLabelsMapStatus = readLabelsMapFile(trainedModelLabelsFullName, labelsMap);
    if (!readLabelsMapStatus.ok())
    {
        CD_ERROR("Fail loading labels \"%s\".\n",trainedModelLabelsFullName.c_str());
        return;
    }
    CD_SUCCESS("Labels \"%s\" loaded correctly.\n",trainedModelLabelsFullName.c_str());
    CD_SUCCESS("%d labels have been loaded.\n",labelsMap.size());;

    valid = true; // setTensorShape at firstArrived
}

/*****************************************************************/

void TensorFlowDetector::setTensorShape(tensorflow::int64 h, tensorflow::int64 w)
{
    CD_DEBUG("%d, %d\n",h,w);
    shape = tensorflow::TensorShape();
    shape.AddDim(1);
    shape.AddDim(h);
    shape.AddDim(w);
    shape.AddDim(3);
}

/*****************************************************************/

bool TensorFlowDetector::detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                                std::vector<DetectedObject*>& detectedObjects,
                                yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret)
{
    if(!firstArrived)
    {
        setTensorShape(inYarpImg.width(),inYarpImg.height());
        firstArrived = true;
    }

    cv::Mat inCvMat = cv::cvarrToMat((IplImage*)inYarpImg.getIplImage());
    cv::cvtColor(inCvMat, inCvMat, cv::COLOR_BGR2RGB);

    // Mat -> Tensor
    tensorflow::Tensor tensor = tensorflow::Tensor(tensorflow::DT_FLOAT, shape);
    tensorflow::Status readTensorStatus = readTensorFromMat(inCvMat, tensor);
    if (!readTensorStatus.ok())
    {
        CD_ERROR("Mat OpenCV -> Tensor: FAIL\n");
        return false;
    }

    // Execute graph
    outputs.clear();
    tensorflow::Status runStatus = session->Run({{inputLayer, tensor}}, outputLayer, {}, &outputs);
    if (!runStatus.ok())
    {
        CD_ERROR("Running model status: FAIL\n");
        return false;
    }

    // Extract results
    tensorflow::TTypes<float>::Flat scores = outputs[1].flat<float>();
    tensorflow::TTypes<float>::Flat classes = outputs[2].flat<float>();
    tensorflow::TTypes<float, 3>::Tensor boxes = outputs[0].flat_outer_dims<float,3>();

    //CD_INFO("scores.size(): %d\n",scores.size()); // 100
    std::vector<size_t> goodIdxs = filterBoxes(scores, boxes, thresholdIOU, thresholdScore);
    CD_INFO("goodIdxs.size(): %d\n",goodIdxs.size());

    for (size_t i = 0; i < goodIdxs.size(); i++)
    {
        CD_SUCCESS("Detection: \"%s\" -> Score: %d\n",labelsMap[classes(goodIdxs[i])].c_str(),scores(goodIdxs[i]));

        DetectedObject* detectedObject = new DetectedObject;
        detectedObject->setBoundingBox(boxes(0,goodIdxs[i],0),
                                       boxes(0,goodIdxs[i],1),
                                       boxes(0,goodIdxs[i],2),
                                       boxes(0,goodIdxs[i],3));
        detectedObjects.push_back(detectedObject);

        //double score_detection=scores(goodIdxs.at(i));
        //std::string class_name=std::string(labelsMap[classes(goodIdxs.at(i))]);
        /*yarp::os::Bottle bottle;
        bottle.clear();
        bottle.addString(" Detection number: ");
        bottle.addInt(goodIdxs.size());
        bottle.addString(" Detection: ");
        bottle.addString(class_name);
        bottle.addString(" Score: ");
        bottle.addDouble(score_detection);*/

        //drawBoundingBoxesOnImage(inCvMat, scores, classes, boxes, labelsMap, goodIdxs);
        //cv::cvtColor(inCvMat, inCvMat, cv::COLOR_BGR2RGB);
    }

    //yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg = inYarpImg;
    //outYarpImg.setExternal(inCvMat.data,inCvMat.size[1],inCvMat.size[0]);

    //outYarpImg = ret;
    return true;
}

// -----------------------------------------------------------------------------

}// namespace roboticslab
