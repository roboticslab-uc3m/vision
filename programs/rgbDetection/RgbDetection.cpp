// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RgbDetection.hpp"

#include <cstdio>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/sig/ImageDraw.h>

#define DEFAULT_SENSOR_DEVICE "remote_grabber"
#define DEFAULT_SENSOR_REMOTE "/grabber"
#define DEFAULT_LOCAL_PREFIX "/rgbDetection"
#define DEFAULT_PERIOD 0.02 // [s]

using namespace roboticslab;

bool RgbDetection::configure(yarp::os::ResourceFinder & rf)
{
    if (rf.check("help"))
    {
        std::printf("RgbDetection options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        std::printf("\t--sensorDevice (device we create, default: \"%s\")\n", DEFAULT_SENSOR_DEVICE);
        std::printf("\t--sensorRemote (if accesing remote, remote port name, default: \"%s\")\n", DEFAULT_SENSOR_REMOTE);
        std::printf("\t--localPrefix (local port name prefix, default: \"%s\")\n", DEFAULT_LOCAL_PREFIX);
        std::printf("\t--period ([s] default: \"%f\")\n", DEFAULT_PERIOD);
        std::printf("\t--detector (detector device)\n");
        return false;
    }

    yDebug() << "RgbDetection config:" << rf.toString();

    auto strSensorDevice = rf.check("sensorDevice", yarp::os::Value(DEFAULT_SENSOR_DEVICE)).asString();
    auto strSensorRemote = rf.check("sensorRemote", yarp::os::Value(DEFAULT_SENSOR_REMOTE)).asString();
    auto strLocalPrefix = rf.check("localPrefix", yarp::os::Value(DEFAULT_LOCAL_PREFIX)).asString();
    auto strDetector = rf.check("detector", yarp::os::Value("")).asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD)).asFloat64();

    yInfo() << "Using --sensorDevice" << strSensorDevice;
    yInfo() << "Using --sensorRemote" << strSensorRemote;
    yInfo() << "Using --localPrefix" << strLocalPrefix;
    yInfo() << "Using --period" << period;
    yInfo() << "Using --detector" << strDetector;

    yarp::os::Property sensorOptions;
    sensorOptions.fromString(rf.toString());
    sensorOptions.put("device", strSensorDevice);
    sensorOptions.put("local", strLocalPrefix);
    sensorOptions.put("remote", strSensorRemote);

    if (!sensorDevice.open(sensorOptions) || !sensorDevice.view(frameGrabber))
    {
        yError() << "Unable to initiate camera device";
        return false;
    }

    yarp::os::Property detectorOptions;
    detectorOptions.fromString(rf.toString());
    detectorOptions.put("device", strDetector);

    if (!detectorDevice.open(detectorOptions) || !detectorDevice.view(iDetector))
    {
        yError() << "Unable to initiate detector device";
        return false;
    }

    if (!statePort.open(strLocalPrefix + "/state:o"))
    {
        yError() << "Unable to open output state port" << statePort.getName();
        return false;
    }

    if (!imagePort.open(strLocalPrefix + "/img:o"))
    {
        yError() << "Unable to open output image port" << imagePort.getName();
        return false;
    }

    statePort.setWriteOnly();
    imagePort.setWriteOnly();

    return true;
}

double RgbDetection::getPeriod()
{
    return period;
}

bool RgbDetection::updateModule()
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> frame;

    if (!frameGrabber->getImage(frame))
    {
        yWarning() << "Frame acquisition failure";
        return true;
    }

    std::vector<yarp::os::Property> detectedObjects;

    if (!iDetector->detect(frame, detectedObjects))
    {
        yWarning() << "Detector failure";
    }

    if (!detectedObjects.empty())
    {
        static const yarp::sig::PixelRgb red(255, 0, 0);

        auto & stateOutput = statePort.prepare();
        stateOutput.clear();

        for (const auto & detectedObject : detectedObjects)
        {
            auto tlx = detectedObject.find("tlx").asInt32();
            auto tly = detectedObject.find("tly").asInt32();
            auto brx = detectedObject.find("brx").asInt32();
            auto bry = detectedObject.find("bry").asInt32();

            yarp::sig::draw::addRectangleOutline(frame,
                                                 red,
                                                 (tlx + brx) / 2,
                                                 (tly + bry) / 2,
                                                 (brx - tlx) / 2,
                                                 (bry - tly) / 2);

            stateOutput.addDict() = detectedObject;
        }

        statePort.write();
    }

    imagePort.prepare() = frame;
    imagePort.write();

    return true;
}

bool RgbDetection::interruptModule()
{
    statePort.interrupt();
    imagePort.interrupt();
    return true;
}

bool RgbDetection::close()
{
    sensorDevice.close();
    detectorDevice.close();
    statePort.close();
    imagePort.close();
    return true;
}
