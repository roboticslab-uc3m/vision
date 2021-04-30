// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RGBD_DETECTION_HPP__
#define __RGBD_DETECTION_HPP__

#include <yarp/conf/version.h>

#if YARP_VERSION_MINOR >= 5
# include <mutex>
# include <vector>
# include <yarp/os/TypedReaderCallback.h>
#endif

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/IntrinsicParams.h>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup rgbdDetection
 * @brief 2.5D detection.
 */
class RgbdDetection
 :
#if YARP_VERSION_MINOR >= 5
   public yarp::os::TypedReaderCallback<yarp::os::Bottle>,
#endif
   public yarp::os::RFModule
{
public:
    ~RgbdDetection()
    { close(); }

    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;
#if YARP_VERSION_MINOR >= 5
    void onRead(yarp::os::Bottle & bot) override;
#endif

private:
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::IRGBDSensor * iRGBDSensor;
    yarp::sig::IntrinsicParams depthIntrinsicParams;

    yarp::dev::PolyDriver detectorDevice;
    IDetector * iDetector;

    yarp::os::BufferedPort<yarp::os::Bottle> statePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imagePort;

#if YARP_VERSION_MINOR >= 5
    yarp::os::BufferedPort<yarp::os::Bottle> cropPort;
    std::vector<std::pair<unsigned int, unsigned int>> cropVertices;
    mutable std::mutex cropMutex;
#endif

    double period;
};

} // namespace roboticslab

#endif // __RGBD_DETECTION_HPP__
