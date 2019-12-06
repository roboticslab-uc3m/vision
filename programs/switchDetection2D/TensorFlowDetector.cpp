// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <time.h>
#include <utility>
#include <vector>

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

#include "SegmentorThread.hpp"
#include "TensorFlowDetectorHelper.hpp"
#include "TensorFlowDetector.hpp"

namespace roboticslab
{

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
}

/*****************************************************************/

yarp::sig::ImageOf<yarp::sig::PixelRgb> TensorFlowDetector::detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg) {

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

    return outYarpImg;
}


TensorFlowDetector::TensorFlowDetector(yarp::os::Searchable* parameters)
{

    std::string trainedModel = DEFAULT_TRAINEDMODEL;
    std::string trainedModelLabels = DEFAULT_TRAINEDMODEL_LABELS;
    std::printf("\t--pbTrainedModel [file.pb] (default: \"%s\")\n", trainedModel.c_str());
    std::printf("\t--pbtxtTrainedModelLabels [file.pbtxt] (default: \"%s\")\n", trainedModelLabels.c_str());


    if(!parameters->check("context"))
    {
        CD_ERROR("**** \"context\" parameter for HaarDetectionTransformation NOT found\n");
        return;
    }
    std::string context = parameters->find("context").asString();

    if(!parameters->check("trainedModel"))
    {
        CD_ERROR("**** \"trainedModel\" parameter for TensorflowDetectionTransformation NOT found\n");
        return;
    }
    trainedModel = parameters->find("trainedModel").asString();
    CD_DEBUG("**** \"trainedModel\" parameter for TensorflowDetectionTransformation found: \"%s\"\n", trainedModel.c_str());

    if(!parameters->check("trainedModelLabels"))
    {
        CD_ERROR("**** \"trainedModelLabels\" parameter for TensorflowDetectionTransformation NOT found\n");
        return;
    }
    trainedModelLabels = parameters->find("trainedModelLabels").asString();
    CD_DEBUG("**** \"trainedModelLabels\" parameter for TensorflowDetectionTransformation found: \"%s\"\n", trainedModelLabels.c_str());


    if(parameters->check("trainedModel"))
    {
        trainedModel = parameters->find("trainedModel").asString();
    }

    if(parameters->check("trainedModelLabels"))
    {
        trainedModelLabels = parameters->find("trainedModelLabels").asString();
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext(context);
    model = rf.findFileByName(trainedModel);
    labels = rf.findFileByName(trainedModelLabels);

   if (model.empty())
   {
       CD_ERROR("No trained model!\n");
       std::exit(1);
   }

   if (labels.empty())
   {
       CD_ERROR("No trained model labels!\n");
       std::exit(1);
   }

   //outPortShape.open("/tensorflowDetection2D/shape");
   //yarp::sig::ImageOf<yarp::sig::PixelRgb> *inYarpImg=outPortShape.read();;
   //j//tensorflowDetector.configuration(model, labels, inYarpImg);


    if(!parameters->check("context"))
    {
        CD_ERROR("**** \"context\" parameter for TensorflowDetectionTransformation NOT found\n");
        return;
    }

    std::string trainedModelFullName = rf.findFileByName(trainedModel);

    if(trainedModelFullName.empty())
    {
        CD_ERROR("**** full path for file NOT found\n");
        return;
    }
    CD_DEBUG("**** full path for file found: \"%s\"\n", trainedModelFullName.c_str());

    std::string trainedModelLabelsFullName = rf.findFileByName(trainedModelLabels);
    if(trainedModelLabelsFullName.empty())
    {
        CD_ERROR("**** full path for file NOT found\n");
        return;
    }
    CD_DEBUG("**** full path for file found: \"%s\"\n", trainedModelLabelsFullName.c_str());



    valid = true;
}

// -----------------------------------------------------------------------------

}// namespace roboticslab
