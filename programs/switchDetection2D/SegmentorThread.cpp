// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ColorDebug.h>
#include "SegmentorThread.hpp"

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
//TensorflowDetection2D tensorflowDetector;

void SegmentorThread::init(yarp::os::ResourceFinder &rf)
{
    int rateMs = DEFAULT_RATE_MS;
    std::string xmlCascade = DEFAULT_XMLCASCADE;
    std::string trainedModel = DEFAULT_TRAINEDMODEL;
    std::string trainedModelLabels = DEFAULT_TRAINEDMODEL_LABELS;
    algorithm = DEFAULT_ALGORITHM;
    locate = DEFAULT_LOCATE;
    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    morphClosing = DEFAULT_MORPH_CLOSING;
    outImage = DEFAULT_OUT_IMAGE;
    outFeatures.fromString(DEFAULT_OUT_FEATURES);  // it's a bottle!!
    outFeaturesFormat = DEFAULT_OUT_FEATURES_FORMAT;
    seeBounding = DEFAULT_SEE_BOUNDING;
    threshold = DEFAULT_THRESHOLD;

    if (rf.check("help"))
    {
        std::printf("SegmentorThread options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        std::printf("\t--rateMs (default: \"%d\")\n", rateMs);

        //haarDetection
        std::printf("\t--xmlCascade [file.xml] (default: \"%s\")\n", xmlCascade.c_str());

        //tensorflowDetection
        std::printf("\t--pbTrainedModel [file.pb] (default: \"%s\")\n", trainedModel.c_str());
        std::printf("\t--pbtxtTrainedModelLabels [file.pbtxt] (default: \"%s\")\n", trainedModelLabels.c_str());

        //colorRegion
        std::printf("\t--algorithm (redMinusBlue,greenMinusRed...; default: \"%s\")\n",algorithm.c_str());
        std::printf("\t--locate (centroid,bottom; default: \"%s\")\n",locate.c_str());
        std::printf("\t--morphClosing (percentage, 2 or 4 okay; default: \"%f\")\n",morphClosing);
        std::printf("\t--maxNumBlobs (default: \"%d\")\n",maxNumBlobs);
        std::printf("\t--outFeatures (default: \"(%s)\")\n",outFeatures.toString().c_str());
        std::printf("\t--outFeaturesFormat (0=bottled,1=minimal; default: \"%d\")\n",outFeaturesFormat);
        std::printf("\t--outImage (0=rgb,1=bw; default: \"%d\")\n",outImage);
        std::printf("\t--seeBounding (0=none,1=contour,2=box,3=both; default: \"%d\")\n",seeBounding);
        std::printf("\t--threshold (default: \"%d\")\n",threshold);

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
        if((strSwitchMode!="haarDetection")&&(strSwitchMode!="tensorflowDetection")&&(strSwitchMode!="colorRegionDetection"))
        {
            std::cout<<strSwitchMode<<" mode not allowed"<<std::endl;
            std::exit(1);
        }
    }

    if(strSwitchMode=="haarDetection")
    {
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

        transformation = new HaarDetectionTransformation(&rf);
        if(!transformation->isValid())
        {
            CD_ERROR("\n");
        }
    }
    else if(strSwitchMode=="colorRegionDetection")
    {
        if (rf.check("algorithm")) algorithm = rf.find("algorithm").asString();
        if (rf.check("locate")) locate = rf.find("locate").asString();
        if (rf.check("maxNumBlobs")) maxNumBlobs = rf.find("maxNumBlobs").asInt32();
        if (rf.check("morphClosing")) morphClosing = rf.find("morphClosing").asFloat64();
        if (rf.check("outFeaturesFormat")) outFeaturesFormat = rf.find("outFeaturesFormat").asInt32();
        std::printf("SegmentorThread using algorithm: %s, locate: %s, maxNumBlobs: %d, morphClosing: %f, outFeaturesFormat: %d.\n",
        algorithm.c_str(),locate.c_str(),maxNumBlobs,morphClosing,outFeaturesFormat);

        if (rf.check("outFeatures"))
        {
            outFeatures = *(rf.find("outFeatures").asList());  // simple overrride
        }
        std::printf("SegmentorThread using outFeatures: (%s).\n", outFeatures.toString().c_str());

        if (rf.check("outImage")) outImage = rf.find("outImage").asInt32();
        if (rf.check("threshold")) threshold = rf.find("threshold").asInt32();
        if (rf.check("seeBounding")) seeBounding = rf.find("seeBounding").asInt32();
        std::printf("SegmentorThread using outImage: %d, rateMs: %d, seeBounding: %d, threshold: %d.\n",
        outImage, rateMs, seeBounding, threshold);
    }
    else if(strSwitchMode=="tensorflowDetection")
    {
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

       outPortShape.open("/tensorflowDetection2D/shape");
       yarp::sig::ImageOf<yarp::sig::PixelRgb> *inYarpImg=outPortShape.read();;
       //j//tensorflowDetector.configuration(model, labels, inYarpImg);
    }

    if (cropSelector != 0)
    {
        cropSelectorProcessor.reset();
        inCropSelectorPort->setReader(cropSelectorProcessor);
    }

    PeriodicThread::setPeriod(rateMs * 0.001);
    PeriodicThread::start();
}

/************************************************************************/

void SegmentorThread::run()
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
    yarp::os::Bottle output;

    if (!camera->getImage(inYarpImg))
    {
        return;
    }

    /*
    if(strSwitchMode=="haarDetection")
    {
        std::cout<<"Executing haarDetection2D..."<<std::endl;
        HaarDetection2D haarDetector;
        outYarpImg=haarDetector.run(inYarpImg, object_cascade);
    }
    else if(strSwitchMode=="colorRegionDetection")
    {
        std::cout<<"Executing ColorRegionDetection2D..."<<std::endl;
        ColorRegionDetection2D colorRegionDetector;
        //    /outYarpImg=/
        colorRegionDetector.run(inYarpImg, algorithm, locate, morphClosing, maxNumBlobs,threshold);
        outYarpImg=colorRegionDetector.outImageProcessed;
        output=colorRegionDetector.outputProcessed;
    }
    else if(strSwitchMode=="tensorflowDetection")
    {
        std::cout<<"Ejecutando tensorflowDetection2D"<<std::endl;
        outYarpImg=tensorflowDetector.run(inYarpImg);
        output=tensorflowDetector.bottle;
    }
    */
    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

    if (output.size() > 0)
    {
        pOutPort->write(output);
    }
}
