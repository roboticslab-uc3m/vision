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

/**
 * @ingroup vision_tests
 * @brief Tests \ref ColorRegionDetector
 */
class HaarDetectorTest : public testing::Test
{

    public:
        virtual void SetUp()
        {
            yarp::os::Property deviceOptions;
            deviceOptions.put("device", "HaarDetector");

            if(!detectorDevice.open(deviceOptions))
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

        virtual void TearDown()
        {
        }


    protected:
        roboticslab::IDetector *iDetector;
        yarp::dev::PolyDriver detectorDevice;
};

TEST_F( HaarDetectorTest, HaarDetector1)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300,200);
    yarpImgRgb.zero();

    std::vector<yarp::os::Property> detectedObjects;
    iDetector->detect(yarpImgRgb,detectedObjects);

    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F( HaarDetectorTest, HaarDetector2)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("HaarDetector");
    std::string faceFullName = rf.findFileByName("tests/face-nc.pgm");
    ASSERT_FALSE(faceFullName.empty());

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    bool ok = yarp::sig::file::read(yarpImgRgb, faceFullName, yarp::sig::file::FORMAT_PGM);
    ASSERT_TRUE(ok);

    std::vector<yarp::os::Property> detectedObjects;
    iDetector->detect(yarpImgRgb,detectedObjects);

    ASSERT_EQ(detectedObjects.size(), 1);

    ASSERT_TRUE(detectedObjects[0].check("tlx"));
    ASSERT_TRUE(detectedObjects[0].check("brx"));
    ASSERT_TRUE(detectedObjects[0].check("tly"));
    ASSERT_TRUE(detectedObjects[0].check("bry"));

    int cx = (detectedObjects[0].find("tlx").asInt32() + detectedObjects[0].find("brx").asInt32()) / 2;
    int cy = (detectedObjects[0].find("tly").asInt32() + detectedObjects[0].find("bry").asInt32()) / 2;

    ASSERT_NEAR(cx, yarpImgRgb.width()/2, 25);
    ASSERT_NEAR(cy, yarpImgRgb.height()/2, 25);
}

}  // namespace roboticslab
