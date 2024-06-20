// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleArucoDetector exampleArucoDetector
 * @brief exampleArucoDetector
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <IDetector.hpp>

int main(int argc, char const *argv[])
{
    yarp::os::Property detectorOptions {{"device", yarp::os::Value("ArucoDetector")}};
    yarp::dev::PolyDriver detectorDevice(detectorOptions);
    roboticslab::IDetector *iDetector;

    if (!detectorDevice.isValid() || !detectorDevice.view(iDetector))
    {
        yError() << "Device not available";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("ArucoDetector");
    std::string qrFullName = rf.findFileByName("tests/rdaruco.png");

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;

    if (!yarp::sig::file::read(yarpImgRgb, qrFullName, yarp::sig::file::FORMAT_PNG))
    {
        yError() << "Image file not available";
        return 1;
    }

    yInfo() << "detect()";

    yarp::os::Bottle detectedObjects;

    if (!iDetector->detect(yarpImgRgb, detectedObjects))
    {
        yError() << "Detector failed";
        return 1;
    }

    for (auto i = 0; i < detectedObjects.size(); i++)
    {
        const auto * detectedObject = detectedObjects.get(i).asDict();

        auto tlX = detectedObject->find("tlx").asInt32();
        auto tlY = detectedObject->find("tly").asInt32();
        auto trX = detectedObject->find("trx").asInt32();
        auto trY = detectedObject->find("try").asInt32();
        auto blX = detectedObject->find("blx").asInt32();
        auto blY = detectedObject->find("bly").asInt32();
        auto brX = detectedObject->find("brx").asInt32();
        auto brY = detectedObject->find("bry").asInt32();
        auto text = detectedObject->find("text").asInt32();

        yInfo("aruco%d [[%d,%d],[%d,%d],[%d,%d],[%d,%d]]: \"%d\"", i, tlX, tlY, trX, trY, brX, brY, blX, blY, text);
    }

    return 0;
}
