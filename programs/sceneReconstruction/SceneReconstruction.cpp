// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SceneReconstruction.hpp"

#include <vector>

#include <yarp/conf/version.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

#include <yarp/sig/Image.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_PREFIX = "/sceneReconstruction";
constexpr auto DEFAULT_PERIOD = 0.02; // [s]
constexpr auto DEFAULT_ALGORITHM = "kinfu";

#if YARP_VERSION_MINOR >= 5
constexpr auto VOCAB_OK = yarp::os::createVocab32('o','k');
constexpr auto VOCAB_FAIL = yarp::os::createVocab32('f','a','i','l');
constexpr auto VOCAB_HELP = yarp::os::createVocab32('h','e','l','p');
constexpr auto VOCAB_CMD_PAUSE = yarp::os::createVocab32('p','a','u','s');
constexpr auto VOCAB_CMD_RESUME = yarp::os::createVocab32('r','s','m');
constexpr auto VOCAB_GET_POSE = yarp::os::createVocab32('g','p','o','s');
constexpr auto VOCAB_GET_POINTS = yarp::os::createVocab32('g','p','c');
constexpr auto VOCAB_GET_POINTS_AND_NORMALS = yarp::os::createVocab32('g','p','c','n');
#else
constexpr auto VOCAB_OK = yarp::os::createVocab('o','k');
constexpr auto VOCAB_FAIL = yarp::os::createVocab('f','a','i','l');
constexpr auto VOCAB_HELP = yarp::os::createVocab('h','e','l','p');
constexpr auto VOCAB_CMD_PAUSE = yarp::os::createVocab('p','a','u','s');
constexpr auto VOCAB_CMD_RESUME = yarp::os::createVocab('r','s','m');
constexpr auto VOCAB_GET_POSE = yarp::os::createVocab('g','p','o','s');
constexpr auto VOCAB_GET_POINTS = yarp::os::createVocab('g','p','c');
constexpr auto VOCAB_GET_POINTS_AND_NORMALS = yarp::os::createVocab('g','p','c','n');
#endif

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

    yarp::os::Bottle makeUsage()
    {
        return {
            yarp::os::Value(VOCAB_HELP, true),
            yarp::os::Value("\tlist commands"),
            yarp::os::Value(VOCAB_CMD_PAUSE, true),
            yarp::os::Value("\tpause scene reconstruction, don't process next frames"),
            yarp::os::Value(VOCAB_CMD_RESUME, true),
            yarp::os::Value("\tstart/resume scene reconstruction, process incoming frames"),
            yarp::os::Value(VOCAB_GET_POSE, true),
            yarp::os::Value("\tretrieve current camera pose"),
            yarp::os::Value(VOCAB_GET_POINTS, true),
            yarp::os::Value("\tretrieve point cloud"),
            yarp::os::Value(VOCAB_GET_POINTS_AND_NORMALS, true),
            yarp::os::Value("\tretrieve point cloud with normals"),
        };
    }
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

    rpcServer.setReader(*this);
    return true;
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

bool SceneReconstruction::read(yarp::os::ConnectionReader & connection)
{
    auto * writer = connection.getWriter();
    yarp::os::Bottle command;

    if (!command.read(connection) || writer == nullptr)
    {
        return false;
    }

    if (command.size() == 0)
    {
        yCWarning(KINFU) << "Got empty bottle";
        yarp::os::Bottle reply {yarp::os::Value(VOCAB_FAIL, true)};
        return reply.write(*writer);
    }

    yCDebug(KINFU) << "command:" << command.toString();

#if YARP_VERSION_MINOR >= 5
    switch (command.get(0).asVocab32())
#else
    switch (command.get(0).asVocab())
#endif
    {
    case VOCAB_HELP:
    {
        static auto usage = makeUsage();
#if YARP_VERSION_MINOR >= 5
        yarp::os::Bottle reply {yarp::os::Value(yarp::os::createVocab32('m','a','n','y'), true)};
#else
        yarp::os::Bottle reply {yarp::os::Value(yarp::os::createVocab('m','a','n','y'), true)};
#endif
        reply.append(usage);
        return reply.write(*writer);
    }
    case VOCAB_CMD_PAUSE:
    {
        isRunning = false;
        yarp::os::Bottle reply {yarp::os::Value(VOCAB_OK, true)};
        return reply.write(*writer);
    }
    case VOCAB_CMD_RESUME:
    {
        isRunning = true;
        yarp::os::Bottle reply {yarp::os::Value(VOCAB_OK, true)};
        return reply.write(*writer);
    }
    case VOCAB_GET_POSE:
    {
        yarp::sig::Matrix pose;
        kinfuMutex.lock();
        kinfu->getPose(pose);
        kinfuMutex.unlock();
        return pose.write(*writer);
    }
    case VOCAB_GET_POINTS:
    {
        yarp::sig::PointCloudXYZ cloud;
        kinfuMutex.lock();
        kinfu->getPoints(cloud);
        kinfuMutex.unlock();
        return cloud.write(*writer);
    }
    case VOCAB_GET_POINTS_AND_NORMALS:
    {
        yarp::sig::PointCloudXYZNormal cloudWithNormals;
        kinfuMutex.lock();
        kinfu->getCloud(cloudWithNormals);
        kinfuMutex.unlock();
        return cloudWithNormals.write(*writer);
    }
    default:
        yarp::os::Bottle reply {yarp::os::Value(VOCAB_FAIL, true)};
        return reply.write(*writer);
    }
}
