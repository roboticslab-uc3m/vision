#include "gtest/gtest.h"

#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <ColorDebug.h>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup vision-tests
 * @brief Tests \ref ColorRegionDetector
 */
class HaarDetectorTest : public testing::Test
{

    public:
        virtual void SetUp()
        {
            yarp::os::Property deviceOptions;
            deviceOptions.put("device", "HaarDetector");
            deviceOptions.put("algorithm", "redMinusGreen");

            if(!detectorDevice.open(deviceOptions))
            {
                CD_ERROR("Failed to open HaarDetector device\n");
                return;
            }

            if (!detectorDevice.view(iDetector))
            {
                CD_ERROR("Problems acquiring detector interface\n");
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
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImage;
    yarpImage.resize(300,200);
    yarpImage.zero();

    std::vector<yarp::os::Property> detectedObjects;

    iDetector->detect(yarpImage,detectedObjects);

    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F( HaarDetectorTest, HaarDetector2)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("HaarDetector");
    std::string faceFullName = rf.findFileByName("tests/face-nc.pgm");
    ASSERT_FALSE(faceFullName.empty());

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImage;
    bool ok = yarp::sig::file::read(yarpImage, faceFullName, yarp::sig::file::FORMAT_PGM);
    ASSERT_TRUE(ok);

    std::vector<yarp::os::Property> detectedObjects;

    iDetector->detect(yarpImage,detectedObjects);

    ASSERT_EQ(detectedObjects.size(), 1);

    ASSERT_TRUE(detectedObjects[0].check("tlx"));
    ASSERT_TRUE(detectedObjects[0].check("brx"));
    ASSERT_TRUE(detectedObjects[0].check("tly"));
    ASSERT_TRUE(detectedObjects[0].check("bry"));

    int cx = (detectedObjects[0].find("tlx").asInt32() + detectedObjects[0].find("brx").asInt32()) / 2;
    int cy = (detectedObjects[0].find("tly").asInt32() + detectedObjects[0].find("bry").asInt32()) / 2;

    ASSERT_NEAR(cx, yarpImage.width()/2, 25);
    ASSERT_NEAR(cy, yarpImage.height()/2, 25);
}

}  // namespace roboticslab
