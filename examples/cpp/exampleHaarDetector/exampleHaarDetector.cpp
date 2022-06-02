// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleHaarDetector exampleHaarDetector
 * @brief exampleHaarDetector
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
    yarp::os::Property detectorOptions {
        {"device", yarp::os::Value("HaarDetector")},
        {"useLBF", yarp::os::Value(true)}
    };

    yarp::dev::PolyDriver detectorDevice(detectorOptions);
    roboticslab::IDetector * iDetector;

    if (!detectorDevice.isValid() || !detectorDevice.view(iDetector))
    {
        yError() << "Device not available";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("HaarDetector");
    std::string faceFullName = rf.findFileByName("tests/face-nc.pgm");

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;

    if (!yarp::sig::file::read(yarpImgRgb, faceFullName, yarp::sig::file::FORMAT_PGM))
    {
        yError() << "Image file not available";
        return 1;
    }

    yInfo() << "detect()";

    yarp::os::Bottle detectedObjects;

    if (!iDetector->detect(yarpImgRgb, detectedObjects) || detectedObjects.size() == 0)
    {
        yError() << "Detector failed";
        return 1;
    }

    const auto * detectedObject = detectedObjects.get(0).asDict();

    yInfo() << "tlx:" << detectedObject->find("tlx").asInt32(); // 90
    yInfo() << "brx:" << detectedObject->find("brx").asInt32(); // 168
    yInfo() << "tly:" << detectedObject->find("tly").asInt32(); // 68
    yInfo() << "bry:" << detectedObject->find("bry").asInt32(); // 14

    const auto * landmarks = detectedObject->find("landmarks").asList();

    if (landmarks)
    {
        yInfo() << "Got" << landmarks->size() << "landmarks";
        yInfo() << landmarks->toString();
    }

    return 0;
}
