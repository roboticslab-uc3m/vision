// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RGBD_DETECTION_HPP__
#define __RGBD_DETECTION_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/IntrinsicParams.h>

#include "IDetector.hpp"
#ifdef HAVE_CROP
# include "YarpCropCallback.hpp"
#endif

namespace roboticslab
{

/**
 * @ingroup rgbdDetection
 * @brief 2.5D detection.
 */
class RgbdDetection : public yarp::os::RFModule
{
public:
    ~RgbdDetection()
    { close(); }

    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;

private:
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::IRGBDSensor * iRGBDSensor;
    yarp::sig::IntrinsicParams depthIntrinsicParams;

    yarp::dev::PolyDriver detectorDevice;
    IDetector * iDetector;

    yarp::os::BufferedPort<yarp::os::Bottle> statePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imagePort;

#if HAVE_CROP
    yarp::os::BufferedPort<yarp::os::Bottle> cropPort;
    YarpCropCallback cropCallback;
#endif

    double period;
};

} // namespace roboticslab

#endif // __RGBD_DETECTION_HPP__
