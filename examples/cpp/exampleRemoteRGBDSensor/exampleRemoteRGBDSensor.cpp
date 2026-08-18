// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleRemoteRGBDSensor exampleRemoteRGBDSensor
 * @brief This example connects to a remote IRGBDSensor device.
 */

#include <cstdio>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>

#include <yarp/sig/Vector.h>

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    std::string strRGBDLocal = "/exampleRemoteRGBDSensor";
    std::string strRGBDRemote = "/rgbd";

    yarp::os::Property options {
        {"device", yarp::os::Value("RGBDSensor_nwc_yarp")},
        {"localImagePort", yarp::os::Value(strRGBDLocal + "/rgbImage:i")},
        {"localDepthPort", yarp::os::Value(strRGBDLocal + "/depthImage:i")},
        {"localRpcPort", yarp::os::Value(strRGBDLocal + "/rpc:o")},
        {"remoteImagePort", yarp::os::Value(strRGBDRemote + "/rgbImage:o")},
        {"remoteDepthPort", yarp::os::Value(strRGBDRemote + "/depthImage:o")},
        {"remoteRpcPort", yarp::os::Value(strRGBDRemote + "/rpc:i")}
    };

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        yError() << "Device not available.";
        return 1;
    }

    yarp::dev::IRGBDSensor *iRGBDSensor;

    if (!dd.view(iRGBDSensor))
    {
        yError() << "Problems acquiring interface";
        return 1;
    }

    yInfo() << "Acquired interface";

    // The following delay should avoid bad status
    yarp::os::SystemClock::delaySystem(1.0);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (yarp::dev::IRGBDSensor::RGBDSensor_status status; iRGBDSensor->getSensorStatus(status) && status == yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE)
#else
    if (yarp::dev::IRGBDSensor::RGBDSensor_status status; (status = iRGBDSensor->getSensorStatus()) == yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE)
#endif
    {
        yInfo() << "Status: RGBD_SENSOR_OK_IN_USE (good!)";
    }
    else
    {
        yWarning() << "Status:" << status << "(bad!)";
    }

    yInfo() << "DepthWidth:" << iRGBDSensor->getDepthWidth();
    yInfo() << "DepthHeight:" << iRGBDSensor->getDepthHeight();
    yInfo() << "RgbWidth:" << iRGBDSensor->getRgbWidth();
    yInfo() << "RgbHeight:" << iRGBDSensor->getRgbHeight();

    return 0;
}
