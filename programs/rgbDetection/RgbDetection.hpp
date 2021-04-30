// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RGB_DETECTION_HPP__
#define __RGB_DETECTION_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#include <yarp/sig/Image.h>

#include "IDetector.hpp"
#include "YarpCropCallback.hpp"

namespace roboticslab
{

/**
 * @ingroup rgbDetection
 * @brief 2D detection.
 */
class RgbDetection : public yarp::os::RFModule
{
public:
    ~RgbDetection()
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    double getPeriod() override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;

private:
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::IFrameGrabberImage * frameGrabber;

    yarp::dev::PolyDriver detectorDevice;
    IDetector * iDetector;

    yarp::os::BufferedPort<yarp::os::Bottle> statePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imagePort;

    yarp::os::BufferedPort<yarp::os::Bottle> cropPort;
    YarpCropCallback cropCallback;

    double period;
};

} // namespace roboticslab

#endif // __RGB_DETECTION_HPP__
