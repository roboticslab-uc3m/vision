#include "gtest/gtest.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

#include "IDetector.hpp"

namespace roboticslab
{

namespace test
{

/**
 * @ingroup vision_tests
 * @brief Tests \ref ColorRegionDetector
 */
class ColorRegionDetectorTest : public testing::Test
{
public:
    void SetUp() override
    {
        yarp::os::Property deviceOptions {
            {"device", yarp::os::Value("ColorRegionDetector")},
            {"algorithm", yarp::os::Value("redMinusGreen")}
        };

        if (!detectorDevice.open(deviceOptions))
        {
            yError() << "Failed to open ColorRegionDetector device";
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

TEST_F(ColorRegionDetectorTest, ColorRegionDetector1)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300, 200);
    yarpImgRgb.zero();

    yarp::os::Bottle detectedObjects;
    iDetector->detect(yarpImgRgb, detectedObjects);

    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F(ColorRegionDetectorTest, ColorRegionDetector2)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300,200);
    yarpImgRgb.zero();
    yarp::sig::draw::addCircle(yarpImgRgb, yarp::sig::PixelRgb(255, 0, 0),
                               yarpImgRgb.width() / 2,yarpImgRgb.height() / 2,
                               yarpImgRgb.height() / 4); // x, y, radius

    yarp::os::Bottle detectedObjects;
    iDetector->detect(yarpImgRgb, detectedObjects);

    ASSERT_EQ(detectedObjects.size(), 1);

    const auto * detectedObject = detectedObjects.get(0).asDict();

    ASSERT_TRUE(detectedObject->check("tlx"));
    ASSERT_TRUE(detectedObject->check("brx"));
    ASSERT_TRUE(detectedObject->check("tly"));
    ASSERT_TRUE(detectedObject->check("bry"));

    int cx = (detectedObject->find("tlx").asInt32() + detectedObject->find("brx").asInt32()) / 2;
    int cy = (detectedObject->find("tly").asInt32() + detectedObject->find("bry").asInt32()) / 2;

    ASSERT_NEAR(cx, yarpImgRgb.width() / 2, 2);
    ASSERT_NEAR(cy, yarpImgRgb.height() / 2, 2);
}

} // namespace test
} // namespace roboticslab
