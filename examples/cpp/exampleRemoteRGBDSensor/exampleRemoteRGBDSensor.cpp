// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleRemoteJr3 exampleRemoteJr3
 * @brief This example connects to a remote @ref Jr3 device.
 */

#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>

#include <yarp/sig/Vector.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

    std::string strRGBDLocal = "/exampleRemoteRGBDSensor";
    std::string strRGBDRemote = "/rgbd";

    yarp::os::Property options;
    options.put("device","RGBDSensorClient");
    options.put("localImagePort",strRGBDLocal+"/rgbImage:i");
    options.put("localDepthPort",strRGBDLocal+"/depthImage:i");
    options.put("localRpcPort",strRGBDLocal+"/rpc:o");
    options.put("remoteImagePort",strRGBDRemote+"/rgbImage:o");
    options.put("remoteDepthPort",strRGBDRemote+"/depthImage:o");
    options.put("remoteRpcPort",strRGBDRemote+"/rpc:i");
    //if(rf.check("noMirror")) options.put("noMirror",1);

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        std::printf("Device not available.\n");
        return 1;
    }

    yarp::dev::IRGBDSensor *iRGBDSensor;

    if (!dd.view(iRGBDSensor))
    {
        std::printf("[error] Problems acquiring interface\n");
        return 1;
    }

    std::printf("[success] acquired interface\n");

    // The following delay should avoid bad status
    yarp::os::Time::delay(1);

    if(iRGBDSensor->getSensorStatus() ==  yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE  )
        std::printf("Status: RGBD_SENSOR_OK_IN_USE (good!)\n");
    else
        std::printf("Status: %d\n", iRGBDSensor->getSensorStatus());

    std::printf("DepthWidth: %d\n", iRGBDSensor->getDepthWidth());
    std::printf("DepthHeight: %d\n", iRGBDSensor->getDepthHeight());
    std::printf("RgbWidth: %d\n", iRGBDSensor->getRgbWidth());
    std::printf("RgbHeight: %d\n", iRGBDSensor->getDepthHeight());

    dd.close();

    return 0;
}
