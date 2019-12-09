// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Bottle.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ColorDebug.h>

#include "ColorRegionDetector.hpp"

namespace roboticslab
{

/*****************************************************************/

ColorRegionDetector::ColorRegionDetector(yarp::os::Searchable* parameters)
{
    algorithm = DEFAULT_ALGORITHM;
    CD_DEBUG("*** \"algorithm\" (default: \"%s\")\n", algorithm.c_str());
    if(parameters->check("algorithm"))
    {
        CD_INFO("**** \"algorithm\" parameter for ColorRegionDetectionTransformation found\n");
        algorithm = parameters->find("algorithm").asString();
    }

    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    if(parameters->check("maxNumBlobs"))
    {
        CD_INFO("**** \"maxNumBlobs\" parameter for ColorRegionDetectionTransformation found\n");
        maxNumBlobs = parameters->find("maxNumBlobs").asInt32();
    }

    morphClosing = DEFAULT_MORPH_CLOSING;
    if(parameters->check("morphClosing"))
    {
        CD_INFO("**** \"morphClosing\" parameter for ColorRegionDetectionTransformation found\n");
        morphClosing = parameters->find("morphClosing").asFloat64();
    }

    outFeaturesFormat = DEFAULT_OUT_FEATURES_FORMAT;
    if(parameters->check("outFeaturesFormat"))
    {
        CD_INFO("**** \"outFeaturesFormat\" parameter for ColorRegionDetectionTransformation found\n");
        outFeaturesFormat = parameters->find("outFeaturesFormat").asInt32();
    }

    outImage = DEFAULT_OUT_IMAGE;
    if(parameters->check("outImage"))
    {
        CD_INFO("**** \"outImage\" parameter for ColorRegionDetectionTransformation found\n");
        outImage = parameters->find("outImage").asInt32();
    }

    threshold = DEFAULT_THRESHOLD;
    if(parameters->check("threshold"))
    {
        CD_INFO("**** \"threshold\" parameter for ColorRegionDetectionTransformation found\n");
        threshold = parameters->find("threshold").asInt32();
    }

    seeBounding = DEFAULT_SEE_BOUNDING;
    if(parameters->check("seeBounding"))
    {
        CD_INFO("**** \"seeBounding\" parameter for ColorRegionDetectionTransformation found\n");
        seeBounding = parameters->find("seeBounding").asInt32();
    }

    printf("DetectorThread using outImage: %d, seeBounding: %d, threshold: %d.\n", outImage, seeBounding, threshold);
    printf("DetectorThread using algorithm: %s, locate: %s, maxNumBlobs: %d, morphClosing: %f, outFeaturesFormat: %d.\n",
            algorithm.c_str(),locate.c_str(),maxNumBlobs,morphClosing,outFeaturesFormat);

    valid = true;
}

/*****************************************************************/

bool ColorRegionDetector::detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                                 std::vector<DetectedObject*>& detectedObjects,
                                 yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret)
{

    // {yarp ImageOf Rgb -> openCv Mat Bgr}
    IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg.width(), inYarpImg.height()),
                                         IPL_DEPTH_8U, 3 );
    cvCvtColor((IplImage*)inYarpImg.getIplImage(), inIplImage, CV_RGB2BGR);
    cv::Mat inCvMat = cv::cvarrToMat(inIplImage);

    // Because Travis stuff goes with [openCv Mat Bgr] for now
    Travis travis(false,true);    // ::Travis(quiet=true, overwrite=true);
    travis.setCvMat(inCvMat);
    if (algorithm=="hue")
        travis.binarize("hue", threshold-5,threshold+5);
    else if(algorithm=="canny")
        travis.binarize("canny");
    else
        travis.binarize(algorithm.c_str(), threshold);
    travis.morphClosing(inYarpImg.width() * morphClosing / 100.0 );
    int numBlobs = travis.blobize(maxNumBlobs);
    if( 0 == numBlobs )
    {
        travis.release();
        return false;
    }
    std::vector<cv::Point2d> blobsXY;
    if( ! travis.getBlobsXY(blobsXY) )
    {
        travis.release();
        return false;
    }
    std::vector<double> blobsAngle,blobsArea,blobsAspectRatio,blobsAxisFirst,blobsAxisSecond,blobsPerimeter;
    std::vector<double> blobsRectangularity,blobsSolidity;
    std::vector<double> blobsHue,blobsSat,blobsVal,blobsHueStdDev,blobsSatStdDev,blobsValStdDev;
    travis.getBlobsArea(blobsArea);
    travis.getBlobsPerimeter(blobsPerimeter);
    travis.getBlobsSolidity(blobsSolidity);
    travis.getBlobsHSV(blobsHue,blobsSat,blobsVal,blobsHueStdDev,blobsSatStdDev,blobsValStdDev);
    if( ! travis.getBlobsAngle(0,blobsAngle) )  // method: 0=box, 1=ellipse; note check for return as 1 can break
    {
        travis.release();
        return false;
    }
    travis.getBlobsAspectRatio(blobsAspectRatio,blobsAxisFirst,blobsAxisSecond);  // must be called after getBlobsAngle!!!!
    travis.getBlobsRectangularity(blobsRectangularity);  // must be called after getBlobsAngle!!!!
    cv::Mat outCvMat = travis.getCvMat(outImage,seeBounding);
    travis.release();

    // { openCv Mat Bgr -> yarp ImageOf Rgb}
    IplImage outIplImage = outCvMat;
    cvCvtColor(&outIplImage,&outIplImage, CV_BGR2RGB);
    char sequence[] = "RGB";
    strcpy (outIplImage.channelSeq,sequence);
    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
    outYarpImg.wrapIplImage(&outIplImage);
    yarp::sig::PixelRgb blue(0,0,255);
    for( int i = 0; i < blobsXY.size(); i++)
        yarp::sig::draw::addCircle(outYarpImg,blue,blobsXY[i].x,blobsXY[i].y,3);
    outImageProcessed=outYarpImg;

    //return outYarpImg; crea conflicto con return; de funciones travis

    // Take advantage we have the travis object and get features for text output
    yarp::os::Bottle output;

    outputProcessed=output;
    //pOutPort->write(output);

    cvReleaseImage( &inIplImage );  // release the memory for the image
    outCvMat.release();  // cvReleaseImage( &outIplImage );  // release the memory for the image


    return true;
}

/************************************************************************/

}  // namespace roboticslab
