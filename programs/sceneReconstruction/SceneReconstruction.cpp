// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SceneReconstruction.hpp"

#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <yarp/sig/Image.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_PREFIX = "/sceneReconstruction";
constexpr auto DEFAULT_PERIOD = 0.02; // [s]
constexpr auto DEFAULT_ALGORITHM = "kinfu";

namespace
{
    template <typename PixelType>
    class TypedRenderUpdater : public RenderUpdater
    {
    public:
        TypedRenderUpdater(KinectFusion & kinfu, std::mutex & mtx, yarp::dev::IRGBDSensor * sensor)
            : RenderUpdater(kinfu, mtx, sensor)
        { renderPort.setWriteOnly(); }

        std::string getPortName() const override
        { return renderPort.getName(); }

        bool openPort(const std::string & name) override
        { return renderPort.open(name); }

        void interruptPort() override
        { renderPort.interrupt(); }

        void closePort() override
        { renderPort.close(); }

        update_result update() override
        { renderPort.write(); return update_result::SUCCESS; }

    protected:
        bool isConnected()
        { return renderPort.getOutputCount() > 0; }

        yarp::sig::ImageOf<PixelType> & prepareFrame()
        { return renderPort.prepare(); }

    private:
        yarp::os::BufferedPort<yarp::sig::ImageOf<PixelType>> renderPort;
    };

    class RenderMonoUpdater : public TypedRenderUpdater<yarp::sig::PixelMono>
    {
    public:
        using TypedRenderUpdater::TypedRenderUpdater;

        update_result update() override
        {
            if (!sensor->getDepthImage(depthFrame))
            {
                return update_result::ACQUISITION_FAILED;
            }

            std::lock_guard<std::mutex> lock(mtx);

            if (!kinfu.update(depthFrame))
            {
                return update_result::KINFU_FAILED;
            }

            if (isConnected())
            {
                kinfu.render(prepareFrame());
                return TypedRenderUpdater::update();
            }

            return update_result::SUCCESS;
        }

    private:
        yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;
    };

    class RenderRgbUpdater : public TypedRenderUpdater<yarp::sig::PixelRgb>
    {
    public:
        using TypedRenderUpdater::TypedRenderUpdater;

        update_result update() override
        {
            if (!sensor->getImages(rgbFrame, depthFrame))
            {
                return update_result::ACQUISITION_FAILED;
            }

            std::lock_guard<std::mutex> lock(mtx);

            if (!kinfu.update(depthFrame, rgbFrame))
            {
                return update_result::KINFU_FAILED;
            }

            if (isConnected())
            {
                kinfu.render(prepareFrame());
                return TypedRenderUpdater::update();
            }

            return update_result::SUCCESS;
        }

    private:
        yarp::sig::FlexImage rgbFrame;
        yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;
    };
}

bool SceneReconstruction::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(KINFU) << "Config:" << rf.toString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD * 1000), "update period (ms)").asInt32() * 0.001;

    auto prefix = rf.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "port prefix").asString();
    yarp::os::Property cameraOptions;

    if (rf.check("remote", "remote RGBD camera port"))
    {
        auto remote = rf.find("remote").asString();
        yCInfo(KINFU) << "Using remote camera at port prefix" << remote;

        cameraOptions = {
            {"device", yarp::os::Value("RGBDSensorClient")},
            {"localImagePort", yarp::os::Value(prefix + "/client/rgbImage:i")},
            {"localDepthPort", yarp::os::Value(prefix + "/client/depthImage:i")},
            {"localRpcPort", yarp::os::Value(prefix + "/client/rpc:o")},
            {"remoteImagePort", yarp::os::Value(remote + "/rgbImage:o")},
            {"remoteDepthPort", yarp::os::Value(remote + "/depthImage:o")},
            {"remoteRpcPort", yarp::os::Value(remote + "/rpc:i")}
        };

        if (rf.check("carrier", "carrier for remote connection to depth camera"))
        {
            cameraOptions.put("DepthCarrier", rf.find("carrier"));
        }
    }
    else
    {
        yCInfo(KINFU) << "Using local camera";
        cameraOptions.fromString(rf.toString());

        if (cameraOptions.check("subdevice"))
        {
            cameraOptions.put("device", cameraOptions.find("subdevice"));
            cameraOptions.unput("subdevice");
        }
    }

    if (!cameraDriver.open(cameraOptions))
    {
        yCError(KINFU) << "Unable to open camera device";
        return false;
    }

    if (!cameraDriver.view(iRGBDSensor))
    {
        yCError(KINFU) << "Unable to acquire RGBD sensor interface handle";
        return false;
    }

    yarp::os::Property depthParams;
    yarp::sig::IntrinsicParams depthIntrinsic;

    if (!iRGBDSensor->getDepthIntrinsicParam(depthParams))
    {
        yCError(KINFU) << "Unable to retrieve depth intrinsic parameters";
        return false;
    }

    depthIntrinsic.fromProperty(depthParams);

    int depthWidth = iRGBDSensor->getDepthWidth();
    int depthHeight = iRGBDSensor->getDepthHeight();

    const auto & params = rf.findGroup("KINECT_FUSION");
    auto algorithm = params.check("algorithm", yarp::os::Value(DEFAULT_ALGORITHM), "algorithm identifier").asString();

    std::vector<std::string> availableAlgorithms {"kinfu"};

#ifdef HAVE_DYNAFU
    availableAlgorithms.push_back("dynafu");
#endif
#ifdef HAVE_KINFU_LS
    availableAlgorithms.push_back("kinfu_ls");
#endif
#ifdef HAVE_COLORED_KINFU
    availableAlgorithms.push_back("colored_kinfu");
#endif

    if (algorithm == "kinfu")
    {
        kinfu = makeKinFu(params, depthIntrinsic, depthWidth, depthHeight);
        renderUpdater = std::make_unique<RenderMonoUpdater>(*kinfu, kinfuMutex, iRGBDSensor);
    }
#ifdef HAVE_DYNAFU
    else if (algorithm == "dynafu")
    {
        kinfu = makeDynaFu(params, depthIntrinsic, depthWidth, depthHeight);
        renderUpdater = std::make_unique<RenderMonoUpdater>(*kinfu, kinfuMutex, iRGBDSensor);
    }
#endif
#ifdef HAVE_KINFU_LS
    else if (algorithm == "kinfu_ls")
    {
        kinfu = makeKinFuLargeScale(params, depthIntrinsic, depthWidth, depthHeight);
        renderUpdater = std::make_unique<RenderMonoUpdater>(*kinfu, kinfuMutex, iRGBDSensor);
    }
#endif
#ifdef HAVE_COLORED_KINFU
    else if (algorithm == "colored_kinfu")
    {
        yarp::os::Property rgbParams;
        yarp::sig::IntrinsicParams rgbIntrinsic;

        if (!iRGBDSensor->getRgbIntrinsicParam(rgbParams))
        {
            yCError(KINFU) << "Unable to retrieve RGB intrinsic parameters";
            return false;
        }

        rgbIntrinsic.fromProperty(rgbParams);

        int rgbWidth = iRGBDSensor->getDepthWidth();
        int rgbHeight = iRGBDSensor->getDepthHeight();

        kinfu = makeColoredKinFu(params, depthIntrinsic, rgbIntrinsic, depthWidth, depthHeight, rgbWidth, rgbHeight);
        renderUpdater = std::make_unique<RenderRgbUpdater>(*kinfu, kinfuMutex, iRGBDSensor);
    }
#endif
    else
    {
        yCError(KINFU) << "Unsupported or unrecognized algorithm" << algorithm << availableAlgorithms;
        return false;
    }

    if (!kinfu || !renderUpdater)
    {
        yCError(KINFU) << "Algorithm or updater handles could not be initialized";
        return false;
    }

    if (!rpcServer.open(prefix + "/rpc:s"))
    {
        yCError(KINFU) << "Unable to open RPC server port" << rpcServer.getName();
        return false;
    }

    if (!renderUpdater->openPort(prefix + "/render:o"))
    {
        yCError(KINFU) << "Unable to open render port" << renderUpdater->getPortName();
        return false;
    }

    return yarp::os::Wire::yarp().attachAsServer(rpcServer);
}

bool SceneReconstruction::updateModule()
{
    if (isRunning && renderUpdater)
    {
        switch (renderUpdater->update())
        {
        case RenderUpdater::update_result::ACQUISITION_FAILED:
            yCWarning(KINFU) << "Unable to retrieve sensor frames";
            break;
        case RenderUpdater::update_result::KINFU_FAILED:
            yCWarning(KINFU) << "Kinect Fusion reset";
            kinfuMutex.lock();
            kinfu.reset();
            kinfuMutex.unlock();
            break;
        case RenderUpdater::update_result::SUCCESS:
            break;
        }
    }

    return true;
}

bool SceneReconstruction::interruptModule()
{
    isRunning = false;

    if (renderUpdater)
    {
        renderUpdater->interruptPort();
    }

    rpcServer.interrupt();
    return true;
}

bool SceneReconstruction::close()
{
    rpcServer.close();

    if (renderUpdater)
    {
        renderUpdater->closePort();
    }

    return cameraDriver.close();
}

void SceneReconstruction::pause()
{
    isRunning = false;
}

void SceneReconstruction::resume()
{
    isRunning = true;
}

return_pose SceneReconstruction::getPose()
{
    yarp::sig::Matrix pose;
    std::lock_guard<std::mutex> lock(kinfuMutex);
    kinfu->getPose(pose);
    return {true, pose};
}

return_points SceneReconstruction::getPoints()
{
    yarp::sig::PointCloudXYZ cloud;
    std::lock_guard<std::mutex> lock(kinfuMutex);
    kinfu->getPoints(cloud);
    return {true, cloud};
}

return_points_with_normals SceneReconstruction::getPointsWithNormals()
{
    yarp::sig::PointCloudXYZNormalRGBA cloudWithNormals;
    std::lock_guard<std::mutex> lock(kinfuMutex);
    kinfu->getCloud(cloudWithNormals);
    return {true, cloudWithNormals};
}
