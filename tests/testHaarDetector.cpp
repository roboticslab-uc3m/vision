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

            yarpImage.resize(300,200);
        }

        virtual void TearDown()
        {
        }


    protected:
        roboticslab::IDetector *iDetector;
        yarp::dev::PolyDriver detectorDevice;
        yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImage;
};

TEST_F( HaarDetectorTest, HaarDetector1)
{
    yarpImage.zero();
    yarp::sig::VectorOf<DetectedObject> detectedObjects;
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

    yarpImage.zero();
    bool ok = yarp::sig::file::read(yarpImage, faceFullName, yarp::sig::file::FORMAT_PGM);
    ASSERT_TRUE(ok);

    yarp::sig::VectorOf<DetectedObject> detectedObjects;
    iDetector->detect(yarpImage,detectedObjects);
    ASSERT_EQ(detectedObjects.size(), 1);

    int cx = (detectedObjects[0]._tlx + detectedObjects[0]._brx) / 2;
    int cy = (detectedObjects[0]._tly + detectedObjects[0]._bry) / 2;

    ASSERT_NEAR(cx, yarpImage.width()/2, 25);
    ASSERT_NEAR(cy, yarpImage.height()/2, 25);
}

}  // namespace roboticslab
