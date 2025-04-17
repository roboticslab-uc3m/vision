// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "QrDetector.hpp"

#include <vector>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/cv/Cv.h>

#include <opencv2/core/version.hpp>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(QR, "rl.QrDetector")
}

bool QrDetector::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(QR) << "Failed to parse parameters";
        return false;
    }

    if (m_epsX > 0.0)
    {
        qrcode.setEpsX(m_epsX);
    }

    if (m_epsY > 0.0)
    {
        qrcode.setEpsY(m_epsY);
    }

    return true;
}

bool QrDetector::detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    std::vector<std::string> texts;
    std::vector<cv::Point> corners;

#if CV_VERSION_MAJOR > 4 || CV_VERSION_MINOR >= 3
    qrcode.detectAndDecodeMulti(inCvMat, texts, corners);
#else
    std::string text = qrcode.detectAndDecode(inCvMat, corners);
    texts.push_back(std::move(text));
#endif

    for (auto i = 0; i < corners.size() / 4; i++)
    {
        const auto & tl = corners[4 * i];
        const auto & tr = corners[4 * i + 1];
        const auto & br = corners[4 * i + 2];
        const auto & bl = corners[4 * i + 3];

        detectedObjects.addDict() = {
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
    }

    return true;
}
