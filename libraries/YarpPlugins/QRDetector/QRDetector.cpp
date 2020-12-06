// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "QRDetector.hpp"

#include <utility>
#include <vector>

#include <yarp/os/Value.h>
#include <yarp/cv/Cv.h>

#include <opencv2/core/types.hpp>
#include <opencv2/core/version.hpp>

using namespace roboticslab;

bool QRDetector::open(yarp::os::Searchable& config)
{
    if (config.check("epsX", "eps used during horizontal scan of QR code stop marker detection"))
    {
        double epsX = config.find("epsX").asFloat64();
        qrcode.setEpsX(epsX);
    }

    if (config.check("epsY", "eps used during vertical scan of QR code stop marker detection"))
    {
        double epsY = config.find("epsY").asFloat64();
        qrcode.setEpsY(epsY);
    }

    return true;
}

bool QRDetector::detect(const yarp::sig::Image& inYarpImg,
                        std::vector<yarp::os::Property>& detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    std::vector<std::string> texts;
    std::vector<cv::Point> corners;

#if CV_MINOR_VERSION >= 3
    qrcode.detectAndDecodeMulti(inCvMat, texts, corners);
#else
    std::string text = qrcode.detectAndDecode(inCvMat, corners);
    texts.push_back(std::move(text));
#endif

    for (auto i = 0; i < texts.size(); i++)
    {
        const auto & tl = corners[4 * i];
        const auto & tr = corners[4 * i + 1];
        const auto & br = corners[4 * i + 2];
        const auto & bl = corners[4 * i + 3];

        yarp::os::Property detectedObject {
            {"tlx", yarp::os::Value(tl.x)},
            {"tly", yarp::os::Value(tl.y)},
            {"trx", yarp::os::Value(tr.x)},
            {"try", yarp::os::Value(tr.y)},
            {"brx", yarp::os::Value(br.x)},
            {"bry", yarp::os::Value(br.y)},
            {"blx", yarp::os::Value(bl.x)},
            {"bly", yarp::os::Value(bl.y)},
            {"text", yarp::os::Value(texts[i])}
        };

        detectedObjects.push_back(std::move(detectedObject));
    }

    return true;
}
