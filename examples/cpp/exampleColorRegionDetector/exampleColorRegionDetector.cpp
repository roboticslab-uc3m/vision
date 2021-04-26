// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleColorRegion exampleColorRegion
 * @brief exampleColorRegion
 */

#include <cstdio>

#include <yarp/os/LogStream.h>
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
        yError() << "Device not available";
        return 1;
    }

    roboticslab::IDetector *iDetector;

    if (!detectorDevice.view(iDetector))
    {
        yError() << "Unable to acquire interface";
        return 1;
    }

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300, 200);
    yarpImgRgb.zero();
    yarp::sig::draw::addCircle(yarpImgRgb, yarp::sig::PixelRgb(255, 0, 0),
                               yarpImgRgb.width() / 2,yarpImgRgb.height() / 2,
                               yarpImgRgb.height() / 4); // x, y, radius

    yInfo() << "detect()";

    yarp::os::Bottle detectedObjects;
    iDetector->detect(yarpImgRgb, detectedObjects);

    const auto * detectedObject = detectedObjects.get(0).asDict();

    yInfo() << detectedObject->find("tlx").asInt32(); // 100
    yInfo() << detectedObject->find("brx").asInt32(); // 201
    yInfo() << detectedObject->find("tly").asInt32(); // 50
    yInfo() << detectedObject->find("bry").asInt32(); // 151

    detectorDevice.close();

    return 0;
}
