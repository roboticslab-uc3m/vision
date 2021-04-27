// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleColorRegion exampleColorRegion
 * @brief exampleColorRegion
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

#include <IDetector.hpp>

int main(int argc, char * argv[])
{
    yarp::os::Property detectorOptions {
        {"device", yarp::os::Value("ColorRegionDetector")},
        {"algorithm", yarp::os::Value("redMinusGreen")}
    };

    yarp::dev::PolyDriver detectorDevice(detectorOptions);
    roboticslab::IDetector * iDetector;

    if (!detectorDevice.isValid() || !detectorDevice.view(iDetector))
    {
        yError() << "Device not available";
        return 1;
    }

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300, 200);
    yarpImgRgb.zero();
    yarp::sig::draw::addCircle(yarpImgRgb, yarp::sig::PixelRgb(255, 0, 0),
                               yarpImgRgb.width() / 2, yarpImgRgb.height() / 2,
                               yarpImgRgb.height() / 4); // x, y, radius

    yInfo() << "detect()";

    yarp::os::Bottle detectedObjects;

    if (!iDetector->detect(yarpImgRgb, detectedObjects) || detectedObjects.size() == 0)
    {
        yError() << "Detector failed";
        return 1;
    }

    const auto * detectedObject = detectedObjects.get(0).asDict();

    yInfo() << "tlx:" << detectedObject->find("tlx").asInt32(); // 100
    yInfo() << "tly:" << detectedObject->find("brx").asInt32(); // 201
    yInfo() << "brx:" << detectedObject->find("tly").asInt32(); // 50
    yInfo() << "bry:" << detectedObject->find("bry").asInt32(); // 151

    return 0;
}
