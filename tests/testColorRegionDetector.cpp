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
        }

        virtual void TearDown()
        {
        }

    protected:
        roboticslab::Detector *detector;
};

TEST_F( ColorRegionDetectorTest, ColorRegionDetector1)
{
}

}  // namespace roboticslab
