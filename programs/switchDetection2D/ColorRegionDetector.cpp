// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Value.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ColorDebug.h>

#include "ColorRegionDetector.hpp"

namespace roboticslab
{

/*****************************************************************/

const std::string ColorRegionDetector::DEFAULT_ALGORITHM = "blueMinusRed";
const double ColorRegionDetector::DEFAULT_MORPH_CLOSING = 2;
const double ColorRegionDetector::DEFAULT_THRESHOLD = 55;

const double ColorRegionDetector::DEFAULT_MAX_NUM_BLOBS = 1;

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
    CD_DEBUG("ColorRegionDetector using algorithm: %s.\n", algorithm.c_str());

    morphClosing = DEFAULT_MORPH_CLOSING;
    if(parameters->check("morphClosing"))
    {
        CD_INFO("**** \"morphClosing\" parameter for ColorRegionDetectionTransformation found\n");
        morphClosing = parameters->find("morphClosing").asFloat64();
    }
    CD_DEBUG("ColorRegionDetector using morphClosing: %f.\n", morphClosing);

    threshold = DEFAULT_THRESHOLD;
    if(parameters->check("threshold"))
    {
        CD_INFO("**** \"threshold\" parameter for ColorRegionDetectionTransformation found\n");
        threshold = parameters->find("threshold").asInt32();
    }
    CD_DEBUG("ColorRegionDetector using threshold: %d.\n", threshold);

    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    if(parameters->check("maxNumBlobs"))
    {
        CD_INFO("**** \"maxNumBlobs\" parameter for ColorRegionDetectionTransformation found\n");
        maxNumBlobs = parameters->find("maxNumBlobs").asInt32();
    }

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
    std::vector<cv::Rect> blobsRect;
    if( ! travis.getBlobsRect(blobsRect) )
    {
        travis.release();
        return false;
    }

    for(size_t i; i<blobsRect.size(); i++)
    {
        cv::Point tl = blobsRect[i].tl();
        cv::Point br = blobsRect[i].br();
        DetectedObject* detectedObject = new DetectedObject;
        detectedObject->setBoundingBox(tl.x,
                                       tl.y,
                                       br.x,
                                       br.y);
        detectedObjects.push_back(detectedObject);
    }

    travis.release();

    return true;
}

/************************************************************************/

}  // namespace roboticslab
