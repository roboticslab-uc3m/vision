// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleQrDetector exampleQrDetector
 * @brief exampleQrDetector
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <IDetector.hpp>

int main(int argc, char * argv[])
{
    yarp::os::Property detectorOptions;
    detectorOptions.put("device", "QrDetector");

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("QrDetector");
    std::string qrFullName = rf.findFileByName("tests/rdqr.png");

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;

    if (!yarp::sig::file::read(yarpImgRgb, qrFullName, yarp::sig::file::FORMAT_PNG))
    {
        yError() << "Image file not available";
        return 1;
    }

    yarp::dev::PolyDriver detectorDevice(detectorOptions);

    if (!detectorDevice.isValid())
    {
        yError() << "Device not available";
        return 1;
    }

    roboticslab::IDetector * iDetector;

    if (!detectorDevice.view(iDetector))
    {
        yError() << "Unable to acquire interfaces";
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
        auto text = detectedObject->find("text").asString();

        yInfo("qr%d [[%d,%d],[%d,%d],[%d,%d],[%d,%d]]: \"%s\"", i, tlX, tlY, trX, trY, brX, brY, blX, blY, text.c_str());
    }

    return 0;
}
