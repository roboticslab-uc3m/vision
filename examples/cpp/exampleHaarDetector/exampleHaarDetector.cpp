// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples_cpp
 * @defgroup exampleHaarDetector exampleHaarDetector
 * @brief exampleHaarDetector
 */

#include <cstdio>
#include <vector>

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

    if(!yarp::sig::file::read(yarpImgRgb, faceFullName, yarp::sig::file::FORMAT_PGM))
    {
        std::printf("Image file not available.\n");
        return 1;
    }

    yarp::dev::PolyDriver detectorDevice(detectorOptions);

    if (!detectorDevice.isValid())
    {
        std::printf("Device not available.\n");
        return 1;
    }

    roboticslab::IDetector *iDetector;

    if (!detectorDevice.view(iDetector))
    {
        std::printf("[error] Problems acquiring interface\n");
        return 1;
    }
    std::printf("[success] acquired interface\n");

    std::printf("detect()\n");

    std::vector<yarp::os::Property> detectedObjects;
    iDetector->detect(yarpImgRgb,detectedObjects);

    std::printf("%d\n", detectedObjects[0].find("tlx").asInt32()); // 90
    std::printf("%d\n", detectedObjects[0].find("brx").asInt32()); // 168
    std::printf("%d\n", detectedObjects[0].find("tly").asInt32()); // 68
    std::printf("%d\n", detectedObjects[0].find("bry").asInt32()); // 14

    detectorDevice.close();

    return 0;
}
