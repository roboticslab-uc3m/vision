// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SceneReconstruction.hpp"

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

constexpr auto VOCAB_HELP = yarp::os::createVocab('h','e','l','p');
constexpr auto VOCAB_CMD_PAUSE = yarp::os::createVocab('p','a','u','s');
constexpr auto VOCAB_CMD_RESUME = yarp::os::createVocab('r','s','m');
constexpr auto VOCAB_GET_CLOUD = yarp::os::createVocab('g','p','c');
constexpr auto VOCAB_GET_NORMALS = yarp::os::createVocab('g','n','r','m');
constexpr auto VOCAB_GET_CLOUD_AND_NORMALS = yarp::os::createVocab('g','p','c','n');

using namespace roboticslab;

namespace
{
    yarp::os::Bottle makeUsage()
    {
        return {
            yarp::os::Value(yarp::os::createVocab('m','a','n','y'), true),
            yarp::os::Value(VOCAB_HELP, true),
            yarp::os::Value("\tlist commands"),
            yarp::os::Value(VOCAB_CMD_PAUSE, true),
            yarp::os::Value("\tpause scene reconstruction, don't process next frames"),
            yarp::os::Value(VOCAB_CMD_RESUME, true),
            yarp::os::Value("\tstart/resume scene reconstruction, process incoming frames"),
            yarp::os::Value(VOCAB_GET_CLOUD, true),
            yarp::os::Value("\tretrieve point cloud"),
            yarp::os::Value(VOCAB_GET_NORMALS, true),
            yarp::os::Value("\tretrieve normals, same order as in point cloud"),
            yarp::os::Value(VOCAB_GET_CLOUD_AND_NORMALS, true),
            yarp::os::Value("\tretrieve cloud and normals alongside each other"),
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
        {"localImagePort", yarp::os::Value(prefix + "/rgbImage:i")},
        {"localDepthPort", yarp::os::Value(prefix + "/depthImage:i")},
        {"localRpcPort", yarp::os::Value(prefix + "/rpc:o")},
        {"remoteImagePort", yarp::os::Value(remote + "/rgbImage:o")},
        {"remoteDepthPort", yarp::os::Value(remote + "/depthImage:o")},
        {"remoteRpcPort", yarp::os::Value(remote + "/rpc:i")}
    };

    if (!cameraDriver.open(cameraOptions))
    {
        CD_ERROR("Unable to open camera device.\n");
        return false;
    }

    cameraDriver.view(iRGBDSensor);

    if (!rpcServer.open(prefix + "/rpc:s"))
    {
        CD_ERROR("Unable to open RPC server port %s.\n", rpcServer.getName().c_str());
        return false;
    }

    if (!renderPort.open(prefix + "/cloud:o"))
    {
        CD_ERROR("Unable to open render port %s.\n", renderPort.getName().c_str());
        return false;
    }

    if (!yarp::os::RFModule::attach(rpcServer))
    {
        CD_ERROR("Unable to attach responder to RPC server port.\n");
        return false;
    }

    return true;
}

bool SceneReconstruction::updateModule()
{
    if (isRunning)
    {
        ;
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

bool SceneReconstruction::respond(const yarp::os::Bottle & command, yarp::os::Bottle & reply)
{
    if (command.size() == 0)
    {
        CD_WARNING("Got empty bottle.\n");
        return false;
    }

    CD_DEBUG("command: %s\n", command.toString().c_str());

    switch (command.get(0).asVocab())
    {
    case VOCAB_HELP:
        static auto usage = makeUsage();
        reply.append(usage);
        return true;
    case VOCAB_CMD_PAUSE:
        isRunning = false;
        return true;
    case VOCAB_CMD_RESUME:
        isRunning = true;
        return true;
    case VOCAB_GET_CLOUD:
    case VOCAB_GET_NORMALS:
    case VOCAB_GET_CLOUD_AND_NORMALS:
        return true;
    default:
        return yarp::os::RFModule::respond(command, reply);
    }
}
