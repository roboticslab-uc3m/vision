// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCENE_RECONSTRUCTION_HPP__
#define __SCENE_RECONSTRUCTION_HPP__

#include <atomic>
#include <mutex>
#include <string>

#include <yarp/os/RpcServer.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>

#include "KinectFusion.hpp"

namespace roboticslab
{

class RenderUpdater
{
public:
    RenderUpdater(KinectFusion & _kinfu, std::mutex & _mtx, yarp::dev::IRGBDSensor * _sensor)
        : kinfu(_kinfu), mtx(_mtx), sensor(_sensor)
    {}

    enum class update_result { ACQUISITION_FAILED, KINFU_FAILED, SUCCESS };

    virtual ~RenderUpdater() = default;
    virtual std::string getPortName() const = 0;
    virtual bool openPort(const std::string & name) = 0;
    virtual void interruptPort() = 0;
    virtual void closePort() = 0;
    virtual update_result update() = 0;

protected:
    KinectFusion & kinfu;
    std::mutex & mtx;
    yarp::dev::IRGBDSensor * sensor;
};

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

    mutable std::mutex kinfuMutex;
    std::unique_ptr<KinectFusion> kinfu {nullptr};

    yarp::dev::PolyDriver cameraDriver;
    yarp::dev::IRGBDSensor * iRGBDSensor {nullptr};

    std::unique_ptr<RenderUpdater> renderUpdater {nullptr};
    yarp::os::RpcServer rpcServer;
};

} // namespace roboticslab

#endif // __SCENE_RECONSTRUCTION_HPP__
