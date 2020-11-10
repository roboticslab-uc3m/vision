// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SceneReconstruction.hpp"

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

using namespace roboticslab;

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
        {"remoteRpcPort", yarp::os::Value(remote + "/rpc:i")}};

    if (!cameraDriver.open(cameraOptions))
    {
        CD_ERROR("Unable to open camera device.\n");
        return false;
    }

    cameraDriver.view(iRGBDSensor);

    if (!outPort.open(prefix + "/cloud:o"))
    {
        CD_ERROR("Unable to open out port %s\n", outPort.getName().c_str());
        return false;
    }

    return true;
}

bool SceneReconstruction::updateModule()
{
    return true;
}

bool SceneReconstruction::interruptModule()
{
    outPort.interrupt();
    return true;
}

bool SceneReconstruction::close()
{
    outPort.close();
    return cameraDriver.close();
}
