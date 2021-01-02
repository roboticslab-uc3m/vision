// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SceneReconstruction.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

constexpr auto VOCAB_OK = yarp::os::createVocab('o','k');
constexpr auto VOCAB_FAIL = yarp::os::createVocab('f','a','i','l');
constexpr auto VOCAB_HELP = yarp::os::createVocab('h','e','l','p');
constexpr auto VOCAB_CMD_PAUSE = yarp::os::createVocab('p','a','u','s');
constexpr auto VOCAB_CMD_RESUME = yarp::os::createVocab('r','s','m');
constexpr auto VOCAB_GET_POSE = yarp::os::createVocab('g','p','o','s');
constexpr auto VOCAB_GET_POINTS = yarp::os::createVocab('g','p','c');
constexpr auto VOCAB_GET_POINTS_AND_NORMALS = yarp::os::createVocab('g','p','c','n');

using namespace roboticslab;

namespace
{
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
    yDebug() << "Config:" << rf.toString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD * 1000), "update period (ms)").asInt32() * 0.001;

    std::string prefix = rf.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "port prefix").asString();
    yarp::os::Property cameraOptions;

    if (rf.check("remote", "remote RGBD camera port"))
    {
        std::string remote = rf.find("remote").asString();
        yInfo() << "Using remote camera at port prefix" << remote;

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
        yInfo() << "Using local camera";
        cameraOptions.fromString(rf.toString());

        if (cameraOptions.check("subdevice"))
        {
            cameraOptions.put("device", cameraOptions.find("subdevice"));
            cameraOptions.unput("subdevice");
        }
    }

    if (!cameraDriver.open(cameraOptions))
    {
        yError() << "Unable to open camera device";
        return false;
    }

    if (!cameraDriver.view(iRGBDSensor))
    {
        yError() << "Unable to acquire RGBD sensor interface handle";
        return false;
    }

    yarp::os::Property depthParams;
    yarp::sig::IntrinsicParams depthIntrinsic;

    if (!iRGBDSensor->getDepthIntrinsicParam(depthParams))
    {
        yError() << "Unable to retrieve depth intrinsic parameters";
        return false;
    }

    depthIntrinsic.fromProperty(depthParams);

    int width = iRGBDSensor->getDepthWidth();
    int height = iRGBDSensor->getDepthHeight();

    const auto & params = rf.findGroup("KINECT_FUSION");
    std::string algorithm = params.check("algorithm", yarp::os::Value(DEFAULT_ALGORITHM), "algorithm identifier").asString();

    if (algorithm == "kinfu")
    {
        kinfu = makeKinFu(params, depthIntrinsic, width, height);
    }
#ifdef HAVE_DYNAFU
    else if (algorithm == "dynafu")
    {
        kinfu = makeDynaFu(params, depthIntrinsic, width, height);
    }
#endif
#ifdef HAVE_KINFU_LS
    else if (algorithm == "kinfu_ls")
    {
        kinfu = makeKinFuLargeScale(params, depthIntrinsic, width, height);
    }
#endif
    else
    {
        yError() << "Unsupported or unrecognized algorithm:" << algorithm << "(available: kinfu, dynafu, kinfu_ls)";
        return false;
    }

    if (!kinfu)
    {
        yError() << "Algorithm handle could not successfully initialize";
        return false;
    }

    if (!rpcServer.open(prefix + "/rpc:s"))
    {
        yError() << "Unable to open RPC server port" << rpcServer.getName();
        return false;
    }

    if (!renderPort.open(prefix + "/render:o"))
    {
        yError() << "Unable to open render port" << renderPort.getName();
        return false;
    }

    rpcServer.setReader(*this);
    return true;
}

bool SceneReconstruction::updateModule()
{
    if (isRunning)
    {
        yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

        if (!iRGBDSensor->getDepthImage(depthFrame))
        {
            yWarning() << "Unable to retrieve depth frame";
            return true;
        }

        kinfuMutex.lock();

        if (!kinfu->update(depthFrame))
        {
            yWarning() << "Kinect Fusion reset";
            kinfu->reset();
        }

        kinfu->render(renderPort.prepare());
        kinfuMutex.unlock();
        renderPort.write();
    }

    return true;
}

bool SceneReconstruction::interruptModule()
{
    isRunning = false;
    renderPort.interrupt();
    rpcServer.interrupt();
    return true;
}

bool SceneReconstruction::close()
{
    rpcServer.close();
    renderPort.close();
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
        yWarning() << "Got empty bottle";
        yarp::os::Bottle reply {yarp::os::Value(VOCAB_FAIL, true)};
        return reply.write(*writer);
    }

    yDebug() << "command:" << command.toString();

    switch (command.get(0).asVocab())
    {
    case VOCAB_HELP:
    {
        static auto usage = makeUsage();
        yarp::os::Bottle reply {yarp::os::Value(yarp::os::createVocab('m','a','n','y'), true)};
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
