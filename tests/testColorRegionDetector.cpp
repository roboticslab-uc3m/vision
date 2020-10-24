#include "gtest/gtest.h"

#include <cmath>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

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
        }

        virtual void TearDown()
        {
        }

    protected:
        roboticslab::IDetector *iDetector;
        yarp::dev::PolyDriver detectorDevice;
};

TEST_F( ColorRegionDetectorTest, ColorRegionDetector1)
{
}

}  // namespace roboticslab
