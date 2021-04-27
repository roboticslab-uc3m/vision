// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RGB_DETECTION_HPP__
#define __RGB_DETECTION_HPP__

#include <mutex>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/TypedReaderCallback.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup rgbDetection
 * @brief 2D detection.
 */
class RgbDetection : public yarp::os::RFModule,
                     public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    ~RgbDetection()
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    double getPeriod() override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;
    void onRead(yarp::os::Bottle & bot) override;

private:
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::IFrameGrabberImage * frameGrabber;

    yarp::dev::PolyDriver detectorDevice;
    IDetector * iDetector;

    yarp::os::BufferedPort<yarp::os::Bottle> statePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imagePort;

    yarp::os::BufferedPort<yarp::os::Bottle> cropPort;
    yarp::sig::VectorOf<std::pair<int, int>> cropVertices;
    mutable std::mutex cropMutex;

    double period;
};

} // namespace roboticslab

#endif // __RGB_DETECTION_HPP__
