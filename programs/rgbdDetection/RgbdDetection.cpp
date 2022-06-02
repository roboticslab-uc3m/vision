// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RgbdDetection.hpp"

#include <cstdio>
#include <tuple>
#include <utility> // std::move
#include <vector>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageUtils.h>

constexpr auto DEFAULT_SENSOR_DEVICE = "RGBDSensorClient";
constexpr auto DEFAULT_SENSOR_REMOTE = "/rgbd";
constexpr auto DEFAULT_LOCAL_PREFIX = "/rgbdDetection";
constexpr auto DEFAULT_PERIOD = 0.02; // [s]

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(RGBD, "rl.RgbdDetection")

    void scaleXY(const yarp::sig::Image & frame1, const yarp::sig::Image & frame2, int px1, int py1, int * px2, int * py2)
    {
        if (frame1.width() != frame2.width() || frame1.height() != frame2.height())
        {
            *px2 = px1 * ((double)frame2.width() / (double)frame1.width());
            *py2 = py1 * ((double)frame2.height() / (double)frame1.height());
        }
        else
        {
            *px2 = px1;
            *py2 = py1;
        }
    }
}

bool RgbdDetection::configure(yarp::os::ResourceFinder &rf)
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

    yCDebug(RGBD) << "Config:" << rf.toString();

    auto strSensorDevice = rf.check("sensorDevice", yarp::os::Value(DEFAULT_SENSOR_DEVICE)).asString();
    auto strSensorRemote = rf.check("sensorRemote", yarp::os::Value(DEFAULT_SENSOR_REMOTE)).asString();
    auto strLocalPrefix = rf.check("localPrefix", yarp::os::Value(DEFAULT_LOCAL_PREFIX)).asString();
    auto strDetector = rf.check("detector", yarp::os::Value("")).asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD)).asFloat64();

    yCInfo(RGBD) << "Using --sensorDevice" << strSensorDevice;
    yCInfo(RGBD) << "Using --sensorRemote" << strSensorRemote;
    yCInfo(RGBD) << "Using --localPrefix" << strLocalPrefix;
    yCInfo(RGBD) << "Using --period" << period;
    yCInfo(RGBD) << "Using --detector" << strDetector;

    yarp::os::Property sensorOptions;
    sensorOptions.fromString(rf.toString());
    sensorOptions.put("device", strSensorDevice);
    sensorOptions.put("localImagePort", strLocalPrefix + "/rgbImage:i");
    sensorOptions.put("localDepthPort", strLocalPrefix + "/depthImage:i");
    sensorOptions.put("localRpcPort", strLocalPrefix + "/rpc:o");
    sensorOptions.put("remoteImagePort", strSensorRemote + "/rgbImage:o");
    sensorOptions.put("remoteDepthPort", strSensorRemote + "/depthImage:o");
    sensorOptions.put("remoteRpcPort", strSensorRemote + "/rpc:i");

    if (!sensorDevice.open(sensorOptions) || !sensorDevice.view(iRGBDSensor))
    {
        yCError(RGBD) << "Unable to initiate camera device";
        return false;
    }

    yarp::os::Property depthParams;

    if (!iRGBDSensor->getDepthIntrinsicParam(depthParams))
    {
        yCError(RGBD) << "Unable to retrieve depth intrinsic parameters";
        return false;
    }

    depthIntrinsicParams.fromProperty(depthParams);

    yarp::os::Property detectorOptions;
    detectorOptions.fromString(rf.toString());
    detectorOptions.put("device", strDetector);

    if (!detectorDevice.open(detectorOptions) || !detectorDevice.view(iDetector))
    {
        yCError(RGBD) << "Unable to initiate detector device";
        return false;
    }

    if (!statePort.open(strLocalPrefix + "/state:o"))
    {
        yCError(RGBD) << "Unable to open output state port" << statePort.getName();
        return false;
    }

    if (!imagePort.open(strLocalPrefix + "/img:o"))
    {
        yCError(RGBD) << "Unable to open output image port" << imagePort.getName();
        return false;
    }

    statePort.setWriteOnly();
    imagePort.setWriteOnly();

    if (!cropPort.open(strLocalPrefix + "/crop:i"))
    {
        yCError(RGBD) << "Unable to open input crop port" << cropPort.getName();
        return false;
    }

    cropPort.setReadOnly();
    cropPort.useCallback(cropCallback);

    return true;
}

double RgbdDetection::getPeriod()
{
    return period;
}

bool RgbdDetection::updateModule()
{
    yarp::sig::FlexImage colorFrame;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

    if (!iRGBDSensor->getImages(colorFrame, depthFrame))
    {
        yCWarning(RGBD) << "Frame acquisition failure";
        return true;
    }

    yarp::sig::ImageOf<yarp::sig::PixelRgb> rgbImage;
    int offsetX = 0;
    int offsetY = 0;

    auto vertices = cropCallback.getVertices();
    bool isRgbCompatible = rgbImage.getPixelCode() == colorFrame.getPixelCode();

    if (vertices.size() != 0)
    {
        if (!yarp::sig::utils::cropRect(colorFrame, vertices[0], vertices[1], rgbImage))
        {
            yCWarning(RGBD) << "Crop failed, using full color frame";
            isRgbCompatible ? rgbImage.move(std::move(colorFrame)) : rgbImage.copy(colorFrame);
        }
        else
        {
            offsetX = vertices[0].first;
            offsetY = vertices[0].second;
        }
    }
    else
    {
        isRgbCompatible ? rgbImage.move(std::move(colorFrame)) : rgbImage.copy(colorFrame);
    }

    yarp::os::Bottle detectedObjects;

    if (!iDetector->detect(rgbImage, detectedObjects))
    {
        yCWarning(RGBD) << "Detector failure";
    }

    if (detectedObjects.size() != 0)
    {
        static const yarp::sig::PixelRgb red(255, 0, 0);
        static const yarp::sig::PixelRgb green(0, 255, 0);

        std::vector<std::tuple<float, int, int, int, int, int, int>> records;
        decltype(records)::value_type * closest = nullptr;

        for (auto i = 0; i < detectedObjects.size(); i++)
        {
            const auto * detectedObject = detectedObjects.get(i).asDict();
            auto tlx = detectedObject->find("tlx").asInt32();
            auto tly = detectedObject->find("tly").asInt32();
            auto brx = detectedObject->find("brx").asInt32();
            auto bry = detectedObject->find("bry").asInt32();

            int pxColor = (tlx + brx) / 2;
            int pyColor = (tly + bry) / 2;

            int pxDepth, pyDepth;
            scaleXY(rgbImage, depthFrame, pxColor + offsetX, pyColor + offsetY, &pxDepth, &pyDepth);
            float depth = depthFrame.pixel(pxDepth, pyDepth);

            if (depth > 0.0f)
            {
                records.emplace_back(depth, pxDepth, pyDepth, pxColor, pyColor, (brx - tlx) / 2, (bry - tly) / 2);

                if (!closest || depth < std::get<0>(*closest))
                {
                    closest = &records.back();
                }
            }

            const auto * landmarks = detectedObject->find("landmarks").asList();

            if (landmarks && imagePort.getOutputCount() > 0)
            {
                for (auto j = 0; j < landmarks->size(); j++)
                {
                    const auto * pair = landmarks->get(j).asList();
                    int lmx = pair->get(0).asInt32();
                    int lmy = pair->get(1).asInt32();
                    yarp::sig::draw::addCircleOutline(rgbImage, {0, 0, 255}, lmx, lmy, 1);
                }
            }
        }

        for (const auto & r : records)
        {
            const yarp::sig::PixelRgb * color = &red;

            if (&r == closest)
            {
                color = &green;

                float z = std::get<0>(r);
                float x = ((std::get<1>(r) - depthIntrinsicParams.principalPointX) * z) / depthIntrinsicParams.focalLengthX;
                float y = ((std::get<2>(r) - depthIntrinsicParams.principalPointY) * z) / depthIntrinsicParams.focalLengthY;

                statePort.prepare() = {yarp::os::Value(x), yarp::os::Value(y), yarp::os::Value(z)};
                statePort.write();
            }

            if (imagePort.getOutputCount() > 0)
            {
                yarp::sig::draw::addRectangleOutline(rgbImage, *color, std::get<3>(r), std::get<4>(r), std::get<5>(r), std::get<6>(r));
            }
        }
    }

    imagePort.prepare() = rgbImage;
    imagePort.write();
    return true;
}

bool RgbdDetection::interruptModule()
{
    statePort.interrupt();
    imagePort.interrupt();
    cropPort.interrupt();
    cropPort.disableCallback();
    return true;
}

bool RgbdDetection::close()
{
    sensorDevice.close();
    detectorDevice.close();
    statePort.close();
    imagePort.close();
    cropPort.close();
    return true;
}
