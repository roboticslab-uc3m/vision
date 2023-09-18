// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCENE_RECONSTRUCTION_HPP__
#define __SCENE_RECONSTRUCTION_HPP__

#include <atomic>
#include <string>

#include <yarp/os/RpcServer.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>

#include "KinectFusion.hpp"
#include "SceneReconstructionIDL.h"

namespace roboticslab
{

class RenderUpdater
{
public:
    enum class update_result { ACQUISITION_FAILED, KINFU_FAILED, SUCCESS };

    RenderUpdater(KinectFusion & _kinfu, yarp::dev::IRGBDSensor * _sensor) : kinfu(_kinfu), sensor(_sensor) {}

    virtual ~RenderUpdater() = default;
    virtual std::string getPortName() const = 0;
    virtual bool openPort(const std::string & name) = 0;
    virtual void interruptPort() = 0;
    virtual void closePort() = 0;
    virtual update_result update() = 0;

protected:
    KinectFusion & kinfu;
    yarp::dev::IRGBDSensor * sensor;
};

/**
 * @ingroup sceneReconstruction
 *
 * @brief Exposes Kinect Fusion as a YARP service via RPC.
 */
class SceneReconstruction : public yarp::os::RFModule,
                            public SceneReconstructionIDL
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

    bool pause() override;

    bool resume() override;

    bool reset() override;

    return_pose getPose() override;

    return_points getPoints() override;

    return_points_with_normals getPointsWithNormals() override;

private:
    double period;
    std::atomic_bool isRunning {false};
    std::unique_ptr<KinectFusion> kinfu {nullptr};

    yarp::dev::PolyDriver cameraDriver;
    yarp::dev::IRGBDSensor * iRGBDSensor {nullptr};

    std::unique_ptr<RenderUpdater> renderUpdater {nullptr};
    yarp::os::RpcServer rpcServer;
};

} // namespace roboticslab

#endif // __SCENE_RECONSTRUCTION_HPP__
