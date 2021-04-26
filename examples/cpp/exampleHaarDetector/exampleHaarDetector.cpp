// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleHaarDetector exampleHaarDetector
 * @brief exampleHaarDetector
 */

#include <cstdio>

#include <yarp/os/LogStream.h>
//#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <IDetector.hpp>

int main(int argc, char *argv[])
{
    /*yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }*/

    yarp::os::Property detectorOptions;
    detectorOptions.put("device","HaarDetector");

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("HaarDetector");
    std::string faceFullName = rf.findFileByName("tests/face-nc.pgm");

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;

    if (!yarp::sig::file::read(yarpImgRgb, faceFullName, yarp::sig::file::FORMAT_PGM))
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

    roboticslab::IDetector *iDetector;

    if (!detectorDevice.view(iDetector))
    {
        yError() << "Unable to acquire interface";
        return 1;
    }

    yInfo() << "detect()";

    yarp::os::Bottle detectedObjects;
    iDetector->detect(yarpImgRgb, detectedObjects);

    const auto * detectedObject = detectedObjects.get(0).asDict();

    yInfo() << detectedObject->find("tlx").asInt32(); // 90
    yInfo() << detectedObject->find("brx").asInt32(); // 168
    yInfo() << detectedObject->find("tly").asInt32(); // 68
    yInfo() << detectedObject->find("bry").asInt32(); // 14

    detectorDevice.close();

    return 0;
}
