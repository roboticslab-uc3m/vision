// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleDnnDetector exampleDnnDetector
 * @brief exampleDnnDetector
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
        {"device", yarp::os::Value("DnnDetector")},
        {"trainedModel", yarp::os::Value("yolov3-tiny/yolov3-tiny.weights")},
        {"configDNNModel", yarp::os::Value("yolov3-tiny/yolov3-tiny.cfg")},
        {"classesTrainedModel", yarp::os::Value("coco-object-categories.txt")}
    };

    yarp::dev::PolyDriver detectorDevice(detectorOptions);
    roboticslab::IDetector * iDetector;

    if (!detectorDevice.isValid() || !detectorDevice.view(iDetector))
    {
        yError() << "Device not available";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("DnnDetector");
    std::string qrFullName = rf.findFileByName("tests/teddy-bear.png");

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;

    if (!yarp::sig::file::read(yarpImgRgb, qrFullName, yarp::sig::file::FORMAT_PNG))
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

    yInfo() << "tlx:" << detectedObject->find("tlx").asInt32(); // 102
    yInfo() << "tly:" << detectedObject->find("brx").asInt32(); // 264
    yInfo() << "brx:" << detectedObject->find("tly").asInt32(); // 21
    yInfo() << "bry:" << detectedObject->find("bry").asInt32(); // 250

    yInfo() << "confidence:" << detectedObject->find("confidence").asFloat64(); // 0.250131
    yInfo() << "category:" << detectedObject->find("category").asString(); // teddy bear <3

    return 0;
}
