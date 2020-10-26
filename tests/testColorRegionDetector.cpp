#include "gtest/gtest.h"

#include <yarp/os/Property.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

#include <ColorDebug.h>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup vision-tests
 * @brief Tests \ref ColorRegionDetector
 */
class ColorRegionDetectorTest : public testing::Test
{

    public:
        virtual void SetUp()
        {
            yarp::os::Property deviceOptions;
            deviceOptions.put("device", "ColorRegionDetector");
            deviceOptions.put("algorithm", "redMinusGreen");

            if(!detectorDevice.open(deviceOptions))
            {
                CD_ERROR("Failed to open ColorRegionDetector device\n");
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

TEST_F( ColorRegionDetectorTest, ColorRegionDetector1)
{
    yarpImage.zero();
    yarp::sig::VectorOf<DetectedObject> detectedObjects;
    iDetector->detect(yarpImage,detectedObjects);
    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F( ColorRegionDetectorTest, ColorRegionDetector2)
{
    yarpImage.zero();
    yarp::sig::draw::addCircle(yarpImage,yarp::sig::PixelRgb(255,0,0),
                               yarpImage.width()/2,yarpImage.height()/2,
                               yarpImage.height()/4); // x, y, radius

    yarp::sig::VectorOf<DetectedObject> detectedObjects;
    iDetector->detect(yarpImage,detectedObjects);
    ASSERT_EQ(detectedObjects.size(), 1);

    int cx = (detectedObjects[0]._tlx + detectedObjects[0]._brx) / 2;
    int cy = (detectedObjects[0]._tly + detectedObjects[0]._bry) / 2;

    ASSERT_NEAR(cx, yarpImage.width()/2, 2);
    ASSERT_NEAR(cy, yarpImage.height()/2, 2);
}

}  // namespace roboticslab
