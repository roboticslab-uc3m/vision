// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/sig/ImageDraw.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ColorDebug.h>
#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <time.h>
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
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/sig/Image.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.hpp>
#include "TensorflowDetector.hpp"

using namespace roboticslab;

/************************************************************************/

void SegmentorThread::setIFrameGrabberImageDriver(yarp::dev::IFrameGrabberImage *_camera)
{
    camera = _camera;
}

/************************************************************************/

void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/

void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/

void SegmentorThread::init(yarp::os::ResourceFinder &rf)
{
    fx_d = DEFAULT_FX_D;
    fy_d = DEFAULT_FY_D;
    cx_d = DEFAULT_CX_D;
    cy_d = DEFAULT_CY_D;

    int rateMs = DEFAULT_RATE_MS;

    std::string trainedModel = DEFAULT_TRAINEDMODEL;
    std::string trainedModelLabels = DEFAULT_TRAINEDMODEL_LABELS;

    if (rf.check("help"))
    {
        std::printf("SegmentorThread options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        std::printf("\t--fx_d (default: \"%f\")\n", fx_d);
        std::printf("\t--fy_d (default: \"%f\")\n", fy_d);
        std::printf("\t--cx_d (default: \"%f\")\n", cx_d);
        std::printf("\t--cy_d (default: \"%f\")\n", cy_d);
        std::printf("\t--rateMs (default: \"%d\")\n", rateMs);
        std::printf("\t--pbTrainedModel [file.pb] (default: \"%s\")\n", trainedModel.c_str());
        std::printf("\t--pbtxtTrainedModelLabels [file.pbtxt] (default: \"%s\")\n", trainedModelLabels.c_str());
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("fx_d"))
    {
        fx_d = rf.find("fx_d").asDouble();
    }

    if (rf.check("fy_d"))
    {
        fy_d = rf.find("fy_d").asDouble();
    }

    if (rf.check("cx_d"))
    {
        cx_d = rf.find("cx_d").asDouble();
    }

    if (rf.check("cy_d"))
    {
        cy_d = rf.find("cy_d").asDouble();
    }

    CD_INFO("Using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n", fx_d, fy_d, cx_d, cy_d);

    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt();
    }

    if (rf.check("trainedModel"))
    {
        trainedModel = rf.find("trainedModel").asString();
    }

    if (rf.check("trainedModelLabels"))
    {
        trainedModelLabels = rf.find("trainedModelLabels").asString();
    }


    if (rf.check("help"))
    {
        std::exit(1);
    }

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
    if (cropSelector != 0)
    {
        processor.reset();
        inCropSelectorPort->setReader(processor);
    }

    RateThread::setRate(rateMs);
    RateThread::start();

  /*  if (!camera->getImage(inYarpImg_2))
    {
        return;
    }*/

  // Path


  inputPort.open("/tensorflowDetection2D/shape");
  yarp::sig::ImageOf<yarp::sig::PixelRgb> *inYarpImg_2 = inputPort.read();
  resultsPort.open("/tensorflowDetection2D/results");

  std::string LABELS = labels;
  std::string GRAPH = model;

  // Set  node names
  inputLayer = "image_tensor:0";
  outputLayer = {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"};

  // Load .pb frozen model
  tensorflow::string graphPath = GRAPH;
  loadGraphStatus = loadGraph(graphPath, &session);
  if (!loadGraphStatus.ok()) {
      std::cout<<"Fail loading graph "<<graphPath<<"."<<std::endl;
  } else
      std::cout<<"Graph "<<graphPath<<" loaded correctly."<<std::endl;


  // Load labels
  labelsMap = std::map<int,std::string>();
  std::cout<<"Labels "<<LABELS<<" are going to be loaded."<<std::endl;
  readLabelsMapStatus = readLabelsMapFile(LABELS, labelsMap);
  if (!readLabelsMapStatus.ok()) {
      std::cout<<"Fail loading labels "<<LABELS<<"."<<std::endl;
  } else
      std::cout<<"Labels "<<LABELS<<" loaded correctly."<<std::endl;
      std::cout<<labelsMap.size()<<" labels have been loaded."<<std::endl;

  time(&start);
  shape = tensorflow::TensorShape();
  shape.AddDim(1);
  shape.AddDim((tensorflow::int64)inYarpImg_2->height());//inYarpImg_2.height()
  shape.AddDim((tensorflow::int64)inYarpImg_2->width());//inYarpImg_2.width()
  shape.AddDim(3);
  std::cout<<"Taking frames..."<<std::endl;

}

/************************************************************************/

void SegmentorThread::run()
{

    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;

    if (!camera->getImage(inYarpImg))
    {
        return;
    }

    cv::Mat inCvMat = cv::cvarrToMat((IplImage*)inYarpImg.getIplImage());
    cv::cvtColor(inCvMat, inCvMat, cv::COLOR_BGR2RGB);
    std::cout << "Frame: " << iFrame << std::endl;

        if (nFrames % (iFrame + 1) == 0) {
            time(&end);
            fps = 1. * nFrames / difftime(end, start);
            time(&start);
        }
        iFrame++;

        // Mat -> Tensor
        tensor = tensorflow::Tensor(tensorflow::DT_FLOAT, shape);
        tensorflow::Status readTensorStatus = readTensorFromMat(inCvMat, tensor);
        if (!readTensorStatus.ok()) {;
            std::cout<<std::endl;
            std::cout<<std::endl;
            std::cout<<"Mat OpenCV -> Tensor : FAIL"<<std::endl;
        }

        // Execute graph
        outputs.clear();
        tensorflow::Status runStatus = session->Run({{inputLayer, tensor}}, outputLayer, {}, &outputs);
        if (!runStatus.ok()) {
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
        for (size_t i = 0; i < goodIdxs.size(); i++){
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

        yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg = inYarpImg;
        outYarpImg.setExternal(inCvMat.data,inCvMat.size[1],inCvMat.size[0]);
        pOutImg->prepare() = outYarpImg;
        pOutImg->write();

    if (bottle.size() > 0)
    {
        resultsPort.write(bottle);
    }
  }
}
