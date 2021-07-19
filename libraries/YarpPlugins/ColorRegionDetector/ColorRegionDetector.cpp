// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/cv/Cv.h>

#include "TravisLib.hpp"

#include "ColorRegionDetector.hpp"

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(CRD, "rl.ColorRegionDetector")
}

constexpr auto DEFAULT_ALGORITHM = "blueMinusRed";
constexpr auto DEFAULT_MORPH_CLOSING = 2;
constexpr auto DEFAULT_THRESHOLD = 55;
constexpr auto DEFAULT_MAX_NUM_BLOBS = 1;

bool ColorRegionDetector::open(yarp::os::Searchable& config)
{
    algorithm = config.check("algorithm", yarp::os::Value(DEFAULT_ALGORITHM)).asString();
    yCDebug(CRD) << "Using algorithm:" << algorithm;

    morphClosing = config.check("morphClosing", yarp::os::Value(DEFAULT_MORPH_CLOSING)).asFloat64();
    yCDebug(CRD) << "Using morphClosing:" << morphClosing;

    threshold = config.check("threshold", yarp::os::Value(DEFAULT_THRESHOLD)).asInt32();
    yCDebug(CRD) << "Using threshold:" << threshold;

    maxNumBlobs = config.check("maxNumBlobs", yarp::os::Value(DEFAULT_MAX_NUM_BLOBS)).asInt32();
    yCDebug(CRD) << "Using maxNumBlobs:" << maxNumBlobs;

    return true;
}

bool ColorRegionDetector::detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);

    Travis travis(false, true);
    travis.setCvMat(yarp::cv::toCvMat(inYarpImgBgr));

    if (algorithm == "hue")
    {
        travis.binarize("hue", threshold - 5, threshold + 5);
    }
    else if (algorithm == "canny")
    {
        travis.binarize("canny");
    }
    else
    {
        travis.binarize(algorithm.c_str(), threshold);
    }

    travis.morphClosing(inYarpImg.width() * morphClosing / 100.0);
    int numBlobs = travis.blobize(maxNumBlobs);

    if (numBlobs == 0)
    {
        travis.release();
        return false;
    }

    std::vector<cv::Rect> blobsRect;

    if (!travis.getBlobsRect(blobsRect))
    {
        travis.release();
        return false;
    }

    for (const auto & blob : blobsRect)
    {
        const auto & tl = blob.tl();
        const auto & br = blob.br();

        detectedObjects.addDict() = {
            {"tlx", yarp::os::Value(tl.x)},
            {"tly", yarp::os::Value(tl.y)},
            {"brx", yarp::os::Value(br.x)},
            {"bry", yarp::os::Value(br.y)}
        };
    }

    travis.release();

    return true;
}
