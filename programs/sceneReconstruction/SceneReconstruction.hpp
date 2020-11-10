// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCENE_RECONSTRUCTION_HPP__
#define __SCENE_RECONSTRUCTION_HPP__

#include <atomic>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>

#define DEFAULT_PREFIX "/sceneReconstruction"
#define DEFAULT_PERIOD 0.02 // [s]
#define DEFAULT_ALGORITHM "kinfu"

namespace roboticslab
{
/**
 * @ingroup sceneReconstruction
 *
 * @brief ...
 */
class SceneReconstruction : public yarp::os::RFModule
{
public:
    SceneReconstruction() : period(DEFAULT_PERIOD), isRunning(false), iRGBDSensor(nullptr)
    {}

    ~SceneReconstruction() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;

    double getPeriod() override
    { return period; }

    bool updateModule() override;

    bool interruptModule() override;

    bool close() override;

    bool respond(const yarp::os::Bottle & command, yarp::os::Bottle & reply) override;

private:
    double period;
    std::atomic_bool isRunning;
    yarp::dev::PolyDriver cameraDriver;
    yarp::dev::IRGBDSensor * iRGBDSensor;
    yarp::os::RpcServer rpcServer;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> renderPort;
};

} // namespace roboticslab

#endif // __SCENE_RECONSTRUCTION_HPP__
