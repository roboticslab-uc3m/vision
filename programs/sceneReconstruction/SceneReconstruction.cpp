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
    class RenderUpdaterBase : public RenderUpdater
    {
    public:
        RenderUpdaterBase(KinectFusion & kinfu, yarp::dev::IRGBDSensor * sensor) : RenderUpdater(kinfu, sensor)
        { renderPort.setWriteOnly(); }

        std::string getPortName() const override final
        { return renderPort.getName(); }

        bool openPort(const std::string & name) override final
        { return renderPort.open(name); }

        void interruptPort() override final
        { renderPort.interrupt(); }

        void closePort() override final
        { renderPort.close(); }

        update_result update() override
        {
            if (renderPort.getOutputCount() > 0)
            {
                auto & frame = renderPort.prepare();
                frame.setPixelCode(getPixelCode());
                kinfu.render(frame);
                renderPort.write();
            }

            return update_result::SUCCESS;
        }

    protected:
        virtual YarpVocabPixelTypesEnum getPixelCode() const = 0;

    private:
        yarp::os::BufferedPort<yarp::sig::FlexImage> renderPort;
    };

    class RenderMonoUpdater : public RenderUpdaterBase
    {
    public:
        using RenderUpdaterBase::RenderUpdaterBase;

        update_result update() override
        {
            if (!sensor->getDepthImage(depthFrame))
            {
                return update_result::ACQUISITION_FAILED;
            }

            if (!kinfu.update(depthFrame))
            {
                return update_result::KINFU_FAILED;
            }

            return RenderUpdaterBase::update();
        }

    private:
        YarpVocabPixelTypesEnum getPixelCode() const override
        { return VOCAB_PIXEL_MONO; }

        yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;
    };

    class RenderColorUpdater : public RenderUpdaterBase
    {
    public:
        using RenderUpdaterBase::RenderUpdaterBase;

        update_result update() override
        {
            if (!sensor->getImages(colorFrame, depthFrame))
            {
                return update_result::ACQUISITION_FAILED;
            }

            if (!kinfu.update(depthFrame, colorFrame))
            {
                return update_result::KINFU_FAILED;
            }

            return RenderUpdaterBase::update();
        }

    private:
        YarpVocabPixelTypesEnum getPixelCode() const override
        { return VOCAB_PIXEL_RGB; }

        yarp::sig::FlexImage colorFrame;
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
        renderUpdater = std::make_unique<RenderMonoUpdater>(*kinfu, iRGBDSensor);
    }
#ifdef HAVE_DYNAFU
    else if (algorithm == "dynafu")
    {
        kinfu = makeDynaFu(params, depthIntrinsic, depthWidth, depthHeight);
        renderUpdater = std::make_unique<RenderMonoUpdater>(*kinfu, iRGBDSensor);
    }
#endif
#ifdef HAVE_KINFU_LS
    else if (algorithm == "kinfu_ls")
    {
        kinfu = makeKinFuLargeScale(params, depthIntrinsic, depthWidth, depthHeight);
        renderUpdater = std::make_unique<RenderMonoUpdater>(*kinfu, iRGBDSensor);
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
        renderUpdater = std::make_unique<RenderColorUpdater>(*kinfu, iRGBDSensor);
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
            kinfu->reset();
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

bool SceneReconstruction::pause()
{
    yCDebug(KINFU) << "Pausing";
    isRunning = false;
    return true;
}

bool SceneReconstruction::resume()
{
    yCDebug(KINFU) << "Resuming";
    isRunning = true;
    return true;
}

bool SceneReconstruction::reset()
{
    yCDebug(KINFU) << "Resetting";
    kinfu->reset();
    return true;
}

return_pose SceneReconstruction::getPose()
{
    yCDebug(KINFU) << "Requesting pose";
    yarp::sig::Matrix pose;
    kinfu->getPose(pose);
    return {true, pose};
}

return_points SceneReconstruction::getPoints()
{
    yCDebug(KINFU) << "Requesting points";
    yarp::sig::PointCloudXYZ cloud;
    kinfu->getPoints(cloud);
    yCDebug(KINFU) << "Got cloud of" << cloud.size() << "points";
    return {true, cloud};
}

return_points_with_normals SceneReconstruction::getPointsWithNormals()
{
    yCDebug(KINFU) << "Requesting points with normals";
    yarp::sig::PointCloudXYZNormalRGBA cloudWithNormals;
    kinfu->getCloud(cloudWithNormals);
    yCDebug(KINFU) << "Got cloud of" << cloudWithNormals.size() << "points with normals";
    return {true, cloudWithNormals};
}
