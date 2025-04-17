#include "gtest/gtest.h"

#include <unordered_set>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include "IDetector.hpp"

namespace roboticslab::test
{

/**
 * @ingroup vision_tests
 * @brief Tests @ref DnnDetector
 */
class DnnDetectorTest : public testing::Test
{
public:
    void SetUp() override
    {
        yarp::os::Property deviceOptions {
            {"device", yarp::os::Value("DnnDetector")},
            {"trainedModel", yarp::os::Value("yolov3-tiny/yolov3-tiny.weights")},
            {"configDNNModel", yarp::os::Value("yolov3-tiny/yolov3-tiny.cfg")},
            {"classesTrainedModel", yarp::os::Value("coco-object-categories.txt")},
            {"backend", yarp::os::Value("default")},
            {"target", yarp::os::Value("cpu")}
        };

        if (!detectorDevice.open(deviceOptions))
        {
            yError() << "Failed to open DnnDetector device";
            return;
        }

        if (!detectorDevice.view(iDetector))
        {
            yError() << "Problems acquiring detector interface";
            return;
        }
    }

    void TearDown() override
    {
    }

protected:
    roboticslab::IDetector * iDetector;
    yarp::dev::PolyDriver detectorDevice;
};

TEST_F(DnnDetectorTest, DnnDetector1)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(352, 288);

    yarp::os::Bottle detectedObjects;
    ASSERT_TRUE(iDetector->detect(yarpImgRgb, detectedObjects));
    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F(DnnDetectorTest, DnnDetector2)
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("DnnDetector");
    std::string qrFullName = rf.findFileByName("tests/teddy-bear.ppm");
    ASSERT_FALSE(qrFullName.empty());

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    ASSERT_TRUE(yarp::sig::file::read(yarpImgRgb, qrFullName, yarp::sig::file::FORMAT_PPM));

    yarp::os::Bottle detectedObjects;
    ASSERT_TRUE(iDetector->detect(yarpImgRgb, detectedObjects));

    ASSERT_EQ(detectedObjects.size(), 1);
    const auto * detectedObject = detectedObjects.get(0).asDict();
    ASSERT_NE(detectedObject, nullptr);

    ASSERT_NEAR(detectedObject->find("tlx").asInt32(), 100, 5);
    ASSERT_NEAR(detectedObject->find("tly").asInt32(), 20, 5);
    ASSERT_NEAR(detectedObject->find("brx").asInt32(), 265, 5);
    ASSERT_NEAR(detectedObject->find("bry").asInt32(), 250, 5);
    ASSERT_NEAR(detectedObject->find("confidence").asFloat64(), 0.25, 0.05);
    ASSERT_EQ(detectedObject->find("category").asString(), "teddy bear");
}

} // namespace roboticslab::test
