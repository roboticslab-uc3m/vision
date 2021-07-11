// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCENE_RECONSTRUCTION_HPP__
#define __SCENE_RECONSTRUCTION_HPP__

#include <atomic>
#include <mutex>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>

#include "KinectFusion.hpp"

namespace roboticslab
{

/**
 * @ingroup sceneReconstruction
 *
 * @brief Exposes Kinect Fusion as a YARP service via RPC.
 */
class SceneReconstruction : public yarp::os::RFModule,
                            private yarp::os::PortReader
{
public:
    ~SceneReconstruction() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;

    double getPeriod() override
    { return period; }

    bool updateModule() override;

    bool interruptModule() override;

    bool close() override;

    bool read(yarp::os::ConnectionReader & reader) override;

private:
    double period;
    std::atomic_bool isRunning {false};
    std::mutex kinfuMutex;
    std::unique_ptr<KinectFusion> kinfu {nullptr};
    yarp::dev::PolyDriver cameraDriver;
    yarp::dev::IRGBDSensor * iRGBDSensor {nullptr};
    yarp::os::RpcServer rpcServer;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> renderPort;
};

} // namespace roboticslab

#endif // __SCENE_RECONSTRUCTION_HPP__
