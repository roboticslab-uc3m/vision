// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>

#include <yarp/os/Bottle.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ColorDebug.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

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
    int rateMs = DEFAULT_RATE_MS;
    std::string xmlCascade = DEFAULT_XMLCASCADE;
    std::string trainedModel = DEFAULT_TRAINEDMODEL;
    std::string trainedModelLabels = DEFAULT_TRAINEDMODEL_LABELS;

    if (rf.check("help"))
    {
        std::printf("SegmentorThread options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        std::printf("\t--rateMs (default: \"%d\")\n", rateMs);
        std::printf("\t--xmlCascade [file.xml] (default: \"%s\")\n", xmlCascade.c_str());
        std::printf("\t--pbTrainedModel [file.pb] (default: \"%s\")\n", trainedModel.c_str());
        std::printf("\t--pbtxtTrainedModelLabels [file.pbtxt] (default: \"%s\")\n", trainedModelLabels.c_str());

        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("help"))
    {
        std::exit(1);
    }


    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt32();
    }

    if (rf.check("switchMode"))
    {
        strSwitchMode = rf.find("switchMode").asString();
        if((strSwitchMode!="haarDetection")&&(strSwitchMode!="tensorflowDetection")&&(strSwitchMode!="colorRegionDetection")){
          std::cout<<strSwitchMode<<" mode not allowed"<<std::endl;
          std::exit(1);
        }
    }

    if(strSwitchMode=="haarDetection"){

      if (rf.check("xmlCascade"))
      {
          xmlCascade = rf.find("xmlCascade").asString();
      }
      std::string cascade = rf.findFileByName(xmlCascade);

      if (cascade.empty() || !object_cascade.load(cascade))
      {
          CD_ERROR("No cascade!\n");
          std::exit(1);
      }
    }else if(strSwitchMode=="colorRegionDetection"){

    }else if(strSwitchMode=="tensorflowDetection"){

      if (rf.check("trainedModel"))
        {
            trainedModel = rf.find("trainedModel").asString();
        }

        if (rf.check("trainedModelLabels"))
        {
            trainedModelLabels = rf.find("trainedModelLabels").asString();
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

    }

    if (cropSelector != 0)
    {
        processor.reset();
        inCropSelectorPort->setReader(processor);
    }

    PeriodicThread::setPeriod(rateMs * 0.001);
    PeriodicThread::start();
}

/************************************************************************/

void SegmentorThread::run()
{

  yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;
  yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;

  if (!camera->getImage(inYarpImg))
  {
      return;
  }

  if(strSwitchMode=="haarDetection"){

    std::cout<<"Executing haarDetection2D..."<<std::endl;
    HaarDetection2D haarDetector;
    outYarpImg=haarDetector.run(inYarpImg, object_cascade);

  }else if(strSwitchMode=="colorRegionDetection"){

std::cout<<"Ejecutando colorRegionDetection2D"<<std::endl;
}else if(strSwitchMode=="tensorflowDetection"){

std::cout<<"Ejecutando tensorflowDetection2D"<<std::endl;
  }

  pOutImg->prepare() = outYarpImg;
  pOutImg->write();

  /*if (output.size() > 0)
  {
      pOutPort->write(output);
  }*/

}
