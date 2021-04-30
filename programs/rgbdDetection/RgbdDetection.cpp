// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RgbdDetection.hpp"

#include <cstdio>
#include <algorithm> // std::max
#include <tuple>
#include <vector>

#include <yarp/conf/version.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/sig/ImageDraw.h>

#if YARP_VERSION_MINOR >= 5
# include <yarp/sig/ImageUtils.h>
#endif

constexpr auto DEFAULT_SENSOR_DEVICE = "RGBDSensorClient";
constexpr auto DEFAULT_SENSOR_REMOTE = "/rgbd";
constexpr auto DEFAULT_LOCAL_PREFIX = "/rgbdDetection";
constexpr auto DEFAULT_PERIOD = 0.02; // [s]

using namespace roboticslab;

namespace
{
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

    yDebug() << "RgbdDetection config:" << rf.toString();

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
    sensorOptions.put("localImagePort", strLocalPrefix + "/rgbImage:i");
    sensorOptions.put("localDepthPort", strLocalPrefix + "/depthImage:i");
    sensorOptions.put("localRpcPort", strLocalPrefix + "/rpc:o");
    sensorOptions.put("remoteImagePort", strSensorRemote + "/rgbImage:o");
    sensorOptions.put("remoteDepthPort", strSensorRemote + "/depthImage:o");
    sensorOptions.put("remoteRpcPort", strSensorRemote + "/rpc:i");

    if (!sensorDevice.open(sensorOptions) || !sensorDevice.view(iRGBDSensor))
    {
        yError() << "Unable to initiate camera device";
        return false;
    }

    yarp::os::Property depthParams;

    if (!iRGBDSensor->getDepthIntrinsicParam(depthParams))
    {
        yError() << "Unable to retrieve depth intrinsic parameters";
        return false;
    }

    depthIntrinsicParams.fromProperty(depthParams);

#if YARP_VERSION_MINOR < 5
    // Wait for the first few frames to arrive. We kept receiving invalid pixel codes
    // from the depthCamera device if started straight away.
    // https://github.com/roboticslab-uc3m/vision/issues/88
    yarp::os::Time::delay(1.0);
#endif

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

#if YARP_VERSION_MINOR >= 5
    if (!cropPort.open(strLocalPrefix + "/crop:i"))
    {
        yError() << "Unable to open input crop port" << cropPort.getName();
        return false;
    }

    cropPort.setReadOnly();
    cropPort.useCallback(*this);
#endif

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
        yWarning() << "Frame acquisition failure";
        return true;
    }

    yarp::sig::ImageOf<yarp::sig::PixelRgb> rgbImage;
    int offsetX = 0;
    int offsetY = 0;

#if YARP_VERSION_MINOR >= 5
    cropMutex.lock();
    auto vertices = cropVertices;
    cropMutex.unlock();

    if (!vertices.empty())
    {
        if (!yarp::sig::utils::cropRect(colorFrame, vertices[0], vertices[1], rgbImage))
        {
            yWarning() << "Crop failed, using full color frame";
            rgbImage.copy(colorFrame);
        }
        else
        {
            // this is why we need to normalize vertices even if cropRect can do it for us
            offsetX = vertices[0].first;
            offsetY = vertices[0].second;
        }
    }
    else
    {
        rgbImage.copy(colorFrame);
    }
#else
    rgbImage.copy(colorFrame);
#endif

    yarp::os::Bottle detectedObjects;

    if (!iDetector->detect(rgbImage, detectedObjects))
    {
        yWarning() << "Detector failure";
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
            scaleXY(colorFrame, depthFrame, pxColor + offsetX, pyColor + offsetY, &pxDepth, &pyDepth);
            float depth = depthFrame.pixel(pxDepth, pyDepth);

            if (depth > 0.0f)
            {
                records.emplace_back(std::forward_as_tuple(depth, pxDepth, pyDepth,
                                                           pxColor, pyColor, (brx - tlx) / 2, (bry - tly) / 2));

                if (!closest || depth < std::get<0>(*closest))
                {
                    closest = &records.back();
                }
            }
        }

        for (const auto & r : records)
        {
            if (&r == closest)
            {
                yarp::sig::draw::addRectangleOutline(rgbImage, green, std::get<3>(r), std::get<4>(r), std::get<5>(r), std::get<6>(r));

                float z = std::get<0>(r);
                float x = ((std::get<1>(r) - depthIntrinsicParams.principalPointX) * z) / depthIntrinsicParams.focalLengthX;
                float y = ((std::get<2>(r) - depthIntrinsicParams.principalPointY) * z) / depthIntrinsicParams.focalLengthY;

                statePort.prepare() = {yarp::os::Value(x), yarp::os::Value(y), yarp::os::Value(z)};
                statePort.write();
            }
            else
            {
                yarp::sig::draw::addRectangleOutline(rgbImage, red, std::get<3>(r), std::get<4>(r), std::get<5>(r), std::get<6>(r));
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
#if YARP_VERSION_MINOR >= 5
    cropPort.interrupt();
    cropPort.disableCallback();
#endif
    return true;
}

bool RgbdDetection::close()
{
    sensorDevice.close();
    detectorDevice.close();
    statePort.close();
    imagePort.close();
#if YARP_VERSION_MINOR >= 5
    cropPort.close();
#endif
    return true;
}

#if YARP_VERSION_MINOR >= 5
void RgbdDetection::onRead(yarp::os::Bottle & bot)
{
    static bool isCropping = false;

    if (bot.size() == 4)
    {
        auto x1 = bot.get(0).asInt32();
        auto y1 = bot.get(1).asInt32();
        auto x2 = bot.get(2).asInt32();
        auto y2 = bot.get(3).asInt32();

        cropMutex.lock();
        cropVertices = {
            {std::min(x1, x2), std::min(y1, y2)}, // left-top corner
            {std::max(x1, x2), std::max(y1, y2)}  // right-bottom corner
        };
        cropMutex.unlock();

        yInfo("Cropping input frames: (x1: %d, y1: %d) (x2: %d, y2: %d)",
              cropVertices[0].first, cropVertices[0].second,
              cropVertices[1].first, cropVertices[1].second);

        isCropping = true;
    }
    else if (isCropping)
    {
        yInfo() << "Crop disabled";

        cropMutex.lock();
        cropVertices.clear();
        cropMutex.unlock();

        isCropping = false;
    }
}
#endif
