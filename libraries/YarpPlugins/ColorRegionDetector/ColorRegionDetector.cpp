// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Value.h>
#include <yarp/cv/Cv.h>

#include "TravisLib.hpp"

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

bool ColorRegionDetector::open(yarp::os::Searchable& config)
{
    algorithm = DEFAULT_ALGORITHM;
    if(config.check("algorithm"))
    {
        CD_INFO("\"algorithm\" parameter found\n");
        algorithm = config.find("algorithm").asString();
    }
    CD_DEBUG("Using \"algorithm\": %s.\n", algorithm.c_str());

    morphClosing = DEFAULT_MORPH_CLOSING;
    if(config.check("morphClosing"))
    {
        CD_INFO("\"morphClosing\" parameter found\n");
        morphClosing = config.find("morphClosing").asFloat64();
    }
    CD_DEBUG("Using \"morphClosing\": %f.\n", morphClosing);

    threshold = DEFAULT_THRESHOLD;
    if(config.check("threshold"))
    {
        CD_INFO("\"threshold\" parameter found\n");
        threshold = config.find("threshold").asInt32();
    }
    CD_DEBUG("Using \"threshold\": %d.\n", threshold);

    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    if(config.check("maxNumBlobs"))
    {
        CD_INFO("\"maxNumBlobs\" parameter found\n");
        maxNumBlobs = config.find("maxNumBlobs").asInt32();
    }
    CD_DEBUG("Using \"maxNumBlobs\": %d.\n", maxNumBlobs);

    return true;
}

/*****************************************************************/

bool ColorRegionDetector::detect(yarp::sig::FlexImage inYarpImg,
                                 std::vector<yarp::os::Property> &detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImgRgb;
    inYarpImgRgb.setExternal(inYarpImg.getRawImage(), inYarpImg.width(), inYarpImg.height());

    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgRgb);

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

        yarp::os::Property detectedObject;
        detectedObject.put("tlx", tl.x);
        detectedObject.put("tly", tl.y);
        detectedObject.put("brx", br.x);
        detectedObject.put("bry", br.y);
        detectedObjects.push_back(detectedObject);
    }

    travis.release();

    return true;
}

/************************************************************************/

}  // namespace roboticslab
