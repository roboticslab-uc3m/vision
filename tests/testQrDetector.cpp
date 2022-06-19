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
 * @brief Tests @ref QrDetector
 */
class QrDetectorTest : public testing::Test
{
public:
    void SetUp() override
    {
        yarp::os::Property deviceOptions {{"device", yarp::os::Value("QrDetector")}};

        if (!detectorDevice.open(deviceOptions))
        {
            yError() << "Failed to open QrDetector device";
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

decltype(QrDetectorTest::expectedValues) QrDetectorTest::expectedValues = {
    {
        {"tlx", yarp::os::Value(7)},
        {"tly", yarp::os::Value(7)},
        {"trx", yarp::os::Value(225)},
        {"try", yarp::os::Value(7)},
        {"brx", yarp::os::Value(225)},
        {"bry", yarp::os::Value(226)},
        {"blx", yarp::os::Value(7)},
        {"bly", yarp::os::Value(226)},
        {"text", yarp::os::Value("1")}
    },
    {
        {"tlx", yarp::os::Value(247)},
        {"tly", yarp::os::Value(7)},
        {"trx", yarp::os::Value(646)},
        {"try", yarp::os::Value(7)},
        {"brx", yarp::os::Value(646)},
        {"bry", yarp::os::Value(406)},
        {"blx", yarp::os::Value(247)},
        {"bly", yarp::os::Value(406)},
        {"text", yarp::os::Value("2")}
    },
    {
        {"tlx", yarp::os::Value(37)},
        {"tly", yarp::os::Value(245)},
        {"trx", yarp::os::Value(187)},
        {"try", yarp::os::Value(245)},
        {"brx", yarp::os::Value(187)},
        {"bry", yarp::os::Value(394)},
        {"blx", yarp::os::Value(37)},
        {"bly", yarp::os::Value(394)},
        {"text", yarp::os::Value("http://asrob.uc3m.es")}
    }
};

TEST_F(QrDetectorTest, QrDetector1)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    yarpImgRgb.resize(300, 200);

    yarp::os::Bottle detectedObjects;
    ASSERT_TRUE(iDetector->detect(yarpImgRgb, detectedObjects));
    ASSERT_EQ(detectedObjects.size(), 0);
}

TEST_F(QrDetectorTest, QrDetector2)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("QrDetector");
    std::string qrFullName = rf.findFileByName("tests/rdqr.png");
    ASSERT_FALSE(qrFullName.empty());

    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImgRgb;
    ASSERT_TRUE(yarp::sig::file::read(yarpImgRgb, qrFullName, yarp::sig::file::FORMAT_PNG));

    yarp::os::Bottle detectedObjects;
    ASSERT_TRUE(iDetector->detect(yarpImgRgb, detectedObjects));

    ASSERT_GE(detectedObjects.size(), 1);
    ASSERT_LE(detectedObjects.size(), 3);

    for (auto i = 0; i < detectedObjects.size(); i++)
    {
        const auto * detectedObject = detectedObjects.get(i).asDict();
        ASSERT_TRUE(expectedValues.find(*detectedObject) != expectedValues.end());
    }
}

} // namespace roboticslab::test
