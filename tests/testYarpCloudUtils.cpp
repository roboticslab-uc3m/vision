#include "gtest/gtest.h"

#include <string>

#include "YarpCloudUtils.hpp"

namespace roboticslab
{

template <typename T>
void populateCloud(yarp::sig::PointCloud<T> & cloud, int iterations, T && point)
{
    for (auto i = 0; i < iterations; i++)
    {
        cloud.push_back(point);
    }
}

/**
 * @ingroup vision-tests
 * @brief Tests @ref YarpCloudUtils.
 */
class YarpCloudUtilsTest : public testing::Test
{
public:
    virtual void SetUp() override
    {
    }

    virtual void TearDown() override
    {
    }

protected:
    static const int iterations;
    static const std::string sink;
};

const int YarpCloudUtilsTest::iterations = 5;
const std::string YarpCloudUtilsTest::sink = "/dev/null";

TEST_F(YarpCloudUtilsTest, savePLY)
{
    const yarp::sig::VectorOf<int> verts {5, 1, 3, 7, 2, 9, 4, 2, 3}; // 9 vertices = 3 faces

    yarp::sig::PointCloudXY xy;
    populateCloud(xy, iterations, {1.5f, 2.5f});
    ASSERT_EQ(xy.size(), iterations);
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xy, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xy, false));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xy, verts, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xy, verts, false));

    yarp::sig::PointCloudXYZ xyz;
    populateCloud(xyz, iterations, {1.5f, 2.5f, 3.5f});
    ASSERT_EQ(xyz.size(), iterations);
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz, false));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz, verts, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz, verts, false));

    yarp::sig::PointCloudNormal normal;
    populateCloud(normal, iterations, {{1.5f, 2.5f, 3.5f}, 4.5f});
    ASSERT_EQ(normal.size(), iterations);
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, normal, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, normal, false));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, normal, verts, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, normal, verts, false));

    yarp::sig::PointCloudXYZRGBA xyz_rgba;
    populateCloud(xyz_rgba, iterations, {{1.5f, 2.5f, 3.5f}, {10, 11, 12, 13}});
    ASSERT_EQ(xyz_rgba.size(), iterations);
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_rgba, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_rgba, false));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_rgba, verts, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_rgba, verts, false));

    yarp::sig::PointCloudXYZI xyzi;
    populateCloud(xyzi, iterations, {{1.5f, 2.5f, 3.5f}, 4.5f});
    ASSERT_EQ(xyzi.size(), iterations);
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyzi, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyzi, false));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyzi, verts, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyzi, verts, false));

    yarp::sig::PointCloudInterestPointXYZ xyz_interest;
    populateCloud(xyz_interest, iterations, {{1.5f, 2.5f, 3.5f}, 4.5f});
    ASSERT_EQ(xyz_interest.size(), iterations);
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_interest, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_interest, false));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_interest, verts, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_interest, verts, false));

    yarp::sig::PointCloudXYZNormal xyz_normal;
    populateCloud(xyz_normal, iterations, {{1.5f, 2.5f, 3.5f}, {4.5f, 5.5f, 6.5f}, 7.5f});
    ASSERT_EQ(xyz_normal.size(), iterations);
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_normal, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_normal, false));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_normal, verts, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_normal, verts, false));

    yarp::sig::PointCloudXYZNormalRGBA xyz_normal_rgba;
    populateCloud(xyz_normal_rgba, iterations, {{1.5f, 2.5f, 3.5f}, {4.5f, 5.5f, 6.5f}, {10, 11, 12, 13, 7.5f}});
    ASSERT_EQ(xyz_normal_rgba.size(), iterations);
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_normal_rgba, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_normal_rgba, false));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_normal_rgba, verts, true));
    ASSERT_TRUE(YarpCloudUtils::savePLY(sink, xyz_normal_rgba, verts, false));
}

} // namespace roboticslab
