// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SceneReconstruction.hpp"

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

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
    CD_DEBUG("config: %s\n", rf.toString().c_str());

    if (!rf.check("remote", "remote RGBD camera port"))
    {
        CD_ERROR("Missing --remote parameter.\n");
        return false;
    }

    std::string remote = rf.find("remote").asString();
    std::string prefix = rf.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "port prefix").asString();
    int periodInMs = rf.check("period", yarp::os::Value(DEFAULT_PERIOD * 1000), "update period (ms)").asInt32();

    period = periodInMs * 0.001;

    yarp::os::Property cameraOptions {
        {"device", yarp::os::Value("RGBDSensorClient")},
        {"localImagePort", yarp::os::Value(prefix + "/client/rgbImage:i")},
        {"localDepthPort", yarp::os::Value(prefix + "/client/depthImage:i")},
        {"localRpcPort", yarp::os::Value(prefix + "/client/rpc:o")},
        {"remoteImagePort", yarp::os::Value(remote + "/rgbImage:o")},
        {"remoteDepthPort", yarp::os::Value(remote + "/depthImage:o")},
        {"remoteRpcPort", yarp::os::Value(remote + "/rpc:i")}
    };

    if (!cameraDriver.open(cameraOptions))
    {
        CD_ERROR("Unable to open camera device.\n");
        return false;
    }

    if (!cameraDriver.view(iRGBDSensor))
    {
        CD_ERROR("Unable to acquire RGBD sensor interface handle.\n");
        return false;
    }

    yarp::os::Property depthParams;
    yarp::sig::IntrinsicParams depthIntrinsic;

    if (!iRGBDSensor->getDepthIntrinsicParam(depthParams))
    {
        CD_ERROR("Unable to retrieve depth intrinsic parameters.\n");
        return false;
    }

    depthIntrinsic.fromProperty(depthParams);

    int width = iRGBDSensor->getDepthWidth();
    int height = iRGBDSensor->getDepthHeight();
    
    std::string algorithm = rf.check("algorithm", yarp::os::Value(DEFAULT_ALGORITHM), "algorithm identifier").asString();

    if (algorithm == "kinfu")
    {
        const auto & config = rf.findGroup("kinfu");
        kinfu = makeKinFu(config, depthIntrinsic, width, height);
    }
    else
    {
        CD_ERROR("Unsupported or unrecognized algorithm: %s (available: kinfu).\n", algorithm.c_str());
        return false;
    }

    if (!kinfu)
    {
        CD_ERROR("Algorithm handle could not successfully initialize.\n");
        return false;
    }

    if (!rpcServer.open(prefix + "/rpc:s"))
    {
        CD_ERROR("Unable to open RPC server port %s.\n", rpcServer.getName().c_str());
        return false;
    }

    if (!renderPort.open(prefix + "/render:o"))
    {
        CD_ERROR("Unable to open render port %s.\n", renderPort.getName().c_str());
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
            CD_ERROR("Unable to retrieve depth frame.\n");
            return true;
        }

        kinfuMutex.lock();

        if (!kinfu->update(depthFrame))
        {
            CD_WARNING("reset\n");
            kinfu->reset();
        }

        auto & rendered = renderPort.prepare();
        kinfu->render(rendered);
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
        CD_WARNING("Got empty bottle.\n");
        yarp::os::Bottle reply {yarp::os::Value(VOCAB_FAIL, true)};
        return reply.write(*writer);
    }

    CD_DEBUG("command: %s\n", command.toString().c_str());

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
