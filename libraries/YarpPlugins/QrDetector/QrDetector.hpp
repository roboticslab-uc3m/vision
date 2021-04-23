// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __QR_DETECTOR_HPP__
#define __QR_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>

#include <opencv2/objdetect.hpp>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup QrDetector
 * @brief Contains roboticslab::QrDetector.
 */
class QrDetector : public yarp::dev::DeviceDriver,
                   public IDetector
{
public:
    bool open(yarp::os::Searchable& config) override;

    bool detect(const yarp::sig::Image& inYarpImg,
                std::vector<yarp::os::Property>& detectedObjects) override;

private:
    cv::QRCodeDetector qrcode;
};

}

#endif // __QR_DETECTOR_HPP__
