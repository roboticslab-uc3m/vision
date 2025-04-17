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

bool ColorRegionDetector::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(CRD) << "Failed to parse parameters";
        return false;
    }

    return true;
}

bool ColorRegionDetector::detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);

    Travis travis(false, true);
    travis.setCvMat(yarp::cv::toCvMat(inYarpImgBgr));

    if (m_algorithm == "hue")
    {
        travis.binarize("hue", m_threshold - 5, m_threshold + 5);
    }
    else if (m_algorithm == "canny")
    {
        travis.binarize("canny");
    }
    else
    {
        travis.binarize(m_algorithm.c_str(), m_threshold);
    }

    travis.morphClosing(inYarpImg.width() * m_morphClosing / 100.0);
    int numBlobs = travis.blobize(m_maxNumBlobs);

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
