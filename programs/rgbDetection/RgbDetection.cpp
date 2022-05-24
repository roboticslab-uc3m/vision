// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RgbDetection.hpp"

#include <cstdio>
#include <string>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#ifdef HAVE_IMGPROC
# include <opencv2/imgproc.hpp>
#else
# include <yarp/sig/ImageDraw.h>
#endif

constexpr auto DEFAULT_SENSOR_DEVICE = "remote_grabber";
constexpr auto DEFAULT_SENSOR_REMOTE = "/grabber";
constexpr auto DEFAULT_LOCAL_PREFIX = "/rgbDetection";
constexpr auto DEFAULT_PERIOD = 0.02; // [s]

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(RGB, "rl.RgbDetection")
}

#ifdef HAVE_IMGPROC
namespace
{
    std::string findLabel(const yarp::os::Searchable & data)
    {
        if (data.check("category") && data.check("confidence"))
        {
            auto confidence = data.find("confidence").asFloat64();
            return data.find("category").asString() + " " + std::to_string(confidence);
        }
        else if (data.check("text"))
        {
            return data.find("text").asString();
        }
        else
        {
            return {};
        }
    }
}
#endif

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

    yCDebug(RGB) << "Config:" << rf.toString();

    auto strSensorDevice = rf.check("sensorDevice", yarp::os::Value(DEFAULT_SENSOR_DEVICE)).asString();
    auto strSensorRemote = rf.check("sensorRemote", yarp::os::Value(DEFAULT_SENSOR_REMOTE)).asString();
    auto strLocalPrefix = rf.check("localPrefix", yarp::os::Value(DEFAULT_LOCAL_PREFIX)).asString();
    auto strDetector = rf.check("detector", yarp::os::Value("")).asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD)).asFloat64();

    yCInfo(RGB) << "Using --sensorDevice" << strSensorDevice;
    yCInfo(RGB) << "Using --sensorRemote" << strSensorRemote;
    yCInfo(RGB) << "Using --localPrefix" << strLocalPrefix;
    yCInfo(RGB) << "Using --period" << period;
    yCInfo(RGB) << "Using --detector" << strDetector;

    yarp::os::Property sensorOptions;
    sensorOptions.fromString(rf.toString());
    sensorOptions.put("device", strSensorDevice);
    sensorOptions.put("local", strLocalPrefix);
    sensorOptions.put("remote", strSensorRemote);

    if (!sensorDevice.open(sensorOptions) || !sensorDevice.view(frameGrabber))
    {
        yCError(RGB) << "Unable to initiate camera device";
        return false;
    }

    yarp::os::Property detectorOptions;
    detectorOptions.fromString(rf.toString());
    detectorOptions.put("device", strDetector);

    if (!detectorDevice.open(detectorOptions) || !detectorDevice.view(iDetector))
    {
        yCError(RGB) << "Unable to initiate detector device";
        return false;
    }

    if (!statePort.open(strLocalPrefix + "/state:o"))
    {
        yCError(RGB) << "Unable to open output state port" << statePort.getName();
        return false;
    }

    if (!imagePort.open(strLocalPrefix + "/img:o"))
    {
        yCError(RGB) << "Unable to open output image port" << imagePort.getName();
        return false;
    }

    if (!cropPort.open(strLocalPrefix + "/crop:i"))
    {
        yCError(RGB) << "Unable to open input crop port" << cropPort.getName();
        return false;
    }

    statePort.setWriteOnly();
    imagePort.setWriteOnly();

    cropPort.setReadOnly();
    cropPort.useCallback(cropCallback);

    return true;
}

double RgbDetection::getPeriod()
{
    return period;
}

bool RgbDetection::updateModule()
{
    auto vertices = cropCallback.getVertices();

    yarp::sig::ImageOf<yarp::sig::PixelRgb> frame;

    if (vertices.size() == 0)
    {
        if (!frameGrabber->getImage(frame))
        {
            yCWarning(RGB) << "Frame acquisition failure";
            return true;
        }
    }
    else
    {
        if (!frameGrabber->getImageCrop(YARP_CROP_RECT, vertices, frame))
        {
            yCWarning(RGB) << "Cropped frame acquisition failure";
            return true;
        }
    }

    yarp::os::Bottle detectedObjects;

    if (!iDetector->detect(frame, detectedObjects))
    {
        yCWarning(RGB) << "Detector failure";
    }

    if (detectedObjects.size() != 0)
    {
        if (imagePort.getOutputCount() > 0)
        {
#ifdef HAVE_IMGPROC
            cv::Mat cvFrame(frame.height(), frame.width(), CV_8UC3, frame.getRawImage(), frame.getRowSize());
#endif
            for (auto i = 0; i < detectedObjects.size(); i++)
            {

                const auto * detectedObject = detectedObjects.get(i).asDict();
                auto tlx = detectedObject->find("tlx").asInt32();
                auto tly = detectedObject->find("tly").asInt32();
                auto brx = detectedObject->find("brx").asInt32();
                auto bry = detectedObject->find("bry").asInt32();


                yarp::os::Bottle * landmarks = detectedObject->find("landmarks").asList();

                if (landmarks)
                {
                    for (auto j = 0; j < landmarks->size(); j++)
                    {
                        yarp::os::Bottle * pair = landmarks->get(j).asList();
                        int lmx = pair->get(0).asInt32();
                        int lmy = pair->get(1).asInt32();
#ifdef HAVE_IMGPROC
                        cv::circle(cvFrame, {lmx, lmy}, 1, {0, 0, 255}, 1);
#else
                        yarp::sig::draw::addCircleOutline(frame, {0,0,255}, lmx, lmy, 1);
#endif
                    }
                }

#ifdef HAVE_IMGPROC
                cv::rectangle(cvFrame, {tlx, tly}, {brx, bry}, {255, 0, 0});
                std::string label = findLabel(*detectedObject);

                if (!label.empty())
                {
                    int base;
                    cv::Size size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
                    int top = cv::max(tly, size.height);
                    cv::rectangle(cvFrame, {tlx, top - size.height}, {tlx + size.width, top + base}, cv::Scalar::all(255), cv::FILLED);
                    cv::putText(cvFrame, label, {tlx, top}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {});
                }
#else
                yarp::sig::draw::addRectangleOutline(frame, {255, 0, 0}, (tlx + brx) / 2, (tly + bry) / 2, (brx - tlx) / 2, (bry - tly) / 2);
#endif
            }
        }

        statePort.prepare() = detectedObjects;
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
    cropPort.interrupt();
    cropPort.disableCallback();
    return true;
}

bool RgbDetection::close()
{
    sensorDevice.close();
    detectorDevice.close();
    statePort.close();
    imagePort.close();
    cropPort.close();
    return true;
}
