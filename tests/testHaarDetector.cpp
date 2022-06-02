#include "gtest/gtest.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include "IDetector.hpp"

namespace roboticslab
{

namespace test
{

/**
 * @ingroup vision_tests
 * @brief Tests \ref ColorRegionDetector
 */
class HaarDetectorTest : public testing::Test
{
public:
    void SetUp() override
    {
        yarp::os::Property deviceOptions {{"device", yarp::os::Value("HaarDetector")}};

        if (!detectorDevice.open(deviceOptions))
        {
            yError() << "Failed to open HaarDetector device";
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

TEST_F(HaarDetectorTest, HaarDetector1)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300, 200);
    yarpImgRgb.zero();

    yarp::os::Bottle detectedObjects;
    iDetector->detect(yarpImgRgb, detectedObjects);

    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F(HaarDetectorTest, HaarDetector2)
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("HaarDetector");
    std::string faceFullName = rf.findFileByName("tests/face-nc.pgm");
    ASSERT_FALSE(faceFullName.empty());

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    bool ok = yarp::sig::file::read(yarpImgRgb, faceFullName, yarp::sig::file::FORMAT_PGM);
    ASSERT_TRUE(ok);

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

    ASSERT_NEAR(cx, yarpImgRgb.width() / 2, 25);
    ASSERT_NEAR(cy, yarpImgRgb.height() / 2, 25);
}

} // namespace test
} // namespace roboticslab
