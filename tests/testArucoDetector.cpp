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

struct PropertyHasher
{
    std::size_t operator()(const yarp::os::Property & prop) const
    {
        return std::hash<std::string>()(prop.toString());
    }
};

struct PropertyComparer
{
    bool operator()(const yarp::os::Property & lhs, const yarp::os::Property & rhs) const
    {
        return lhs.toString() == rhs.toString();
    }
};

/**
 * @ingroup vision_tests
 * @brief Tests @ref ArucoDetector
 */
class ArucoDetectorTest : public testing::Test
{
public:
    void SetUp() override
    {
        yarp::os::Property deviceOptions {{"device", yarp::os::Value("ArucoDetector")}};

        if (!detectorDevice.open(deviceOptions))
        {
            yError() << "Failed to open ArucoDetector device";
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
    static std::unordered_set<yarp::os::Property, PropertyHasher, PropertyComparer> expectedValues;
};

decltype(ArucoDetectorTest::expectedValues) ArucoDetectorTest::expectedValues = {
    {
        {"tlx", yarp::os::Value(431)},
        {"tly", yarp::os::Value(1164)},
        {"trx", yarp::os::Value(904)},
        {"try", yarp::os::Value(1165)},
        {"brx", yarp::os::Value(903)},
        {"bry", yarp::os::Value(1637)},
        {"blx", yarp::os::Value(430)},
        {"bly", yarp::os::Value(1636)},
        {"id", yarp::os::Value(5)}
    },
    {
        {"tlx", yarp::os::Value(1301)},
        {"tly", yarp::os::Value(515)},
        {"trx", yarp::os::Value(2341)},
        {"try", yarp::os::Value(515)},
        {"brx", yarp::os::Value(2340)},
        {"bry", yarp::os::Value(1557)},
        {"blx", yarp::os::Value(1300)},
        {"bly", yarp::os::Value(1555)},
        {"id", yarp::os::Value(20)}
    },
    {
        {"tlx", yarp::os::Value(334)},
        {"tly", yarp::os::Value(298)},
        {"trx", yarp::os::Value(997)},
        {"try", yarp::os::Value(299)},
        {"brx", yarp::os::Value(996)},
        {"bry", yarp::os::Value(962)},
        {"blx", yarp::os::Value(333)},
        {"bly", yarp::os::Value(961)},
        {"id", yarp::os::Value(10)}
    }
};

TEST_F(ArucoDetectorTest, ArucoDetector1)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300, 200);

    yarp::os::Bottle detectedObjects;
    ASSERT_TRUE(iDetector->detect(yarpImgRgb, detectedObjects));
    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F(ArucoDetectorTest, ArucoDetector2)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("ArucoDetector");
    std::string arucoFullName = rf.findFileByName("tests/aruco.png");
    ASSERT_FALSE(arucoFullName.empty());

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    ASSERT_TRUE(yarp::sig::file::read(yarpImgRgb, arucoFullName, yarp::sig::file::FORMAT_PNG));

    yarp::os::Bottle detectedObjects;
    ASSERT_TRUE(iDetector->detect(yarpImgRgb, detectedObjects));

    ASSERT_GE(detectedObjects.size(), 1);
    ASSERT_LE(detectedObjects.size(), 3);

    yarp::os::Property dict;

    for (auto i = 0; i < detectedObjects.size(); i++)
    {
        const auto * detectedObject = detectedObjects.get(i).asList();
        dict.fromString(detectedObject->toString());
        ASSERT_TRUE(expectedValues.find(dict) != expectedValues.end());
    }
}

} // namespace roboticslab::test
