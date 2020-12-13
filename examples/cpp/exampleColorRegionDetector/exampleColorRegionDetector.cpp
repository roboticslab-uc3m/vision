// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleColorRegion exampleColorRegion
 * @brief exampleColorRegion
 */

#include <cstdio>
#include <vector>

//#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

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
    detectorOptions.put("device","ColorRegionDetector");
    detectorOptions.put("algorithm", "redMinusGreen");

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

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300,200);
    yarpImgRgb.zero();
    yarp::sig::draw::addCircle(yarpImgRgb,yarp::sig::PixelRgb(255,0,0),
                               yarpImgRgb.width()/2,yarpImgRgb.height()/2,
                               yarpImgRgb.height()/4); // x, y, radius

    std::printf("detect()\n");

    std::vector<yarp::os::Property> detectedObjects;
    iDetector->detect(yarpImgRgb,detectedObjects);

    std::printf("%d\n", detectedObjects[0].find("tlx").asInt32()); // 100
    std::printf("%d\n", detectedObjects[0].find("brx").asInt32()); // 201
    std::printf("%d\n", detectedObjects[0].find("tly").asInt32()); // 50
    std::printf("%d\n", detectedObjects[0].find("bry").asInt32()); // 151

    detectorDevice.close();

    return 0;
}
