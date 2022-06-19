#include "gtest/gtest.h"

#include <cmath>
#include <cstdlib>
#include <string>

#include "YarpCloudUtils.hpp"

#ifndef PLY_PATH
# error "missing compile definition PLY_PATH"
#endif

#ifdef YCU_HAVE_PCL
# define _ASSERT_TRUE(cond) ASSERT_TRUE(cond)
# define _ASSERT_FALSE(cond) ASSERT_FALSE(cond)
#else
# define _ASSERT_TRUE(cond) ASSERT_FALSE(cond)
# define _ASSERT_FALSE(cond) ASSERT_FALSE(cond)
#endif

namespace roboticslab::test
{

template <typename T, typename Fn>
void populateCloud(yarp::sig::PointCloud<T> & cloud, int iterations, Fn && generator)
{
    for (auto i = 0; i < iterations; i++)
    {
        cloud.push_back(generator());
    }
}

template <typename Fn>
void populateIndices(yarp::sig::VectorOf<int> & indices, int faces, Fn && generator)
{
    indices.reserve(faces * 3);

    for (auto i = 0; i < faces * 3; i++)
    {
        indices.push_back(generator());
    }
}

inline unsigned char rnu()
{
    return static_cast<unsigned char>(std::rand());
}

inline float rnf()
{
    return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

template <typename T>
bool testEquality(const yarp::sig::PointCloud<T> & cloud1, const yarp::sig::PointCloud<T> & cloud2)
{
    if (cloud1.size() != cloud2.size())
    {
        return false;
    }

    for (auto i = 0; i < cloud1.size(); i++)
    {
        auto b1 = cloud1.toBottle();
        auto b2 = cloud2.toBottle();

        if (b1.size() != b2.size())
        {
            return false;
        }

        for (auto j = 0; j < b1.size(); j++)
        {
            auto diff = std::abs(b1.get(j).asFloat32() - b2.get(j).asFloat32());

            if (diff > 1e-6)
            {
                return false;
            }
        }
    }

    return true;
}

/**
 * @ingroup vision_tests
 * @brief Tests @ref YarpCloudUtils.
 */
class YarpCloudUtilsTest : public testing::Test
{
public:
    void SetUp() override
    {
    }

    void TearDown() override
    {
    }

protected:
    static const int n_points;
    static const int n_faces;
    static const std::string path;
};

const int YarpCloudUtilsTest::n_points = 25;
const int YarpCloudUtilsTest::n_faces = 10;
const std::string YarpCloudUtilsTest::path = PLY_PATH;

TEST_F(YarpCloudUtilsTest, PLY_XY)
{
    using point_t = yarp::sig::DataXY;

    yarp::sig::PointCloud<point_t> cloud_out, cloud_in1, cloud_in2, cloud_in3, cloud_in4;
    yarp::sig::VectorOf<int> indices_out, indices_in1, indices_in2;

    populateIndices(indices_out, n_faces, std::rand);
    ASSERT_EQ(indices_out.size(), n_faces * 3);

    populateCloud(cloud_out, n_points, []() -> point_t { return {rnf(), rnf()}; });
    ASSERT_EQ(cloud_out.size(), n_points);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xy_cloud_binary.ply", cloud_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xy_cloud_binary.ply", cloud_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in1));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xy_cloud_ascii.ply", cloud_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xy_cloud_ascii.ply", cloud_in2));
    ASSERT_TRUE(cloud_out.toString() == cloud_in2.toString());

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xy_mesh_binary.ply", cloud_out, indices_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xy_mesh_binary.ply", cloud_in3, indices_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in3));
    ASSERT_TRUE(indices_in1 == indices_out);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xy_mesh_ascii.ply", cloud_out, indices_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xy_mesh_ascii.ply", cloud_in4, indices_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in4));
    ASSERT_TRUE(indices_in2 == indices_out);
}

TEST_F(YarpCloudUtilsTest, PLY_XYZ)
{
    using point_t = yarp::sig::DataXYZ;

    yarp::sig::PointCloud<point_t> cloud_out, cloud_in1, cloud_in2, cloud_in3, cloud_in4;
    yarp::sig::VectorOf<int> indices_out, indices_in1, indices_in2;

    populateIndices(indices_out, n_faces, std::rand);
    ASSERT_EQ(indices_out.size(), n_faces * 3);

    populateCloud(cloud_out, n_points, []() -> point_t { return {rnf(), rnf(), rnf()}; });
    ASSERT_EQ(cloud_out.size(), n_points);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_cloud_binary.ply", cloud_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_cloud_binary.ply", cloud_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in1));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_cloud_ascii.ply", cloud_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_cloud_ascii.ply", cloud_in2));
    ASSERT_TRUE(cloud_out.toString() == cloud_in2.toString());

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_mesh_binary.ply", cloud_out, indices_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_mesh_binary.ply", cloud_in3, indices_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in3));
    ASSERT_TRUE(indices_in1 == indices_out);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_mesh_ascii.ply", cloud_out, indices_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_mesh_ascii.ply", cloud_in4, indices_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in4));
    ASSERT_TRUE(indices_in2 == indices_out);
}

TEST_F(YarpCloudUtilsTest, PLY_Normal)
{
    using point_t = yarp::sig::DataNormal;

    yarp::sig::PointCloud<point_t> cloud_out, cloud_in1, cloud_in2, cloud_in3, cloud_in4;
    yarp::sig::VectorOf<int> indices_out, indices_in1, indices_in2;

    populateIndices(indices_out, n_faces, std::rand);
    ASSERT_EQ(indices_out.size(), n_faces * 3);

    populateCloud(cloud_out, n_points, []() -> point_t { return {{rnf(), rnf(), rnf()}, rnf()}; });
    ASSERT_EQ(cloud_out.size(), n_points);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "normal_cloud_binary.ply", cloud_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "normal_cloud_binary.ply", cloud_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in1));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "normal_cloud_ascii.ply", cloud_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "normal_cloud_ascii.ply", cloud_in2));
    ASSERT_TRUE(cloud_out.toString() == cloud_in2.toString());

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "normal_mesh_binary.ply", cloud_out, indices_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "normal_mesh_binary.ply", cloud_in3, indices_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in3));
    ASSERT_TRUE(indices_in1 == indices_out);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "normal_mesh_ascii.ply", cloud_out, indices_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "normal_mesh_ascii.ply", cloud_in4, indices_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in4));
    ASSERT_TRUE(indices_in2 == indices_out);
}

TEST_F(YarpCloudUtilsTest, PLY_XYZ_RGBA)
{
    using point_t = yarp::sig::DataXYZRGBA;

    yarp::sig::PointCloud<point_t> cloud_out, cloud_in1, cloud_in2, cloud_in3, cloud_in4;
    yarp::sig::VectorOf<int> indices_out, indices_in1, indices_in2;

    populateIndices(indices_out, n_faces, std::rand);
    ASSERT_EQ(indices_out.size(), n_faces * 3);

    populateCloud(cloud_out, n_points, []() -> point_t { return {{rnf(), rnf(), rnf()}, {rnu(), rnu(), rnu(), rnu()}}; });
    ASSERT_EQ(cloud_out.size(), n_points);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_rgba_cloud_binary.ply", cloud_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_rgba_cloud_binary.ply", cloud_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in1));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_rgba_cloud_ascii.ply", cloud_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_rgba_cloud_ascii.ply", cloud_in2));
    ASSERT_TRUE(cloud_out.toString() == cloud_in2.toString());

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_rgba_mesh_binary.ply", cloud_out, indices_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_rgba_mesh_binary.ply", cloud_in3, indices_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in3));
    ASSERT_TRUE(indices_in1 == indices_out);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_rgba_mesh_ascii.ply", cloud_out, indices_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_rgba_mesh_ascii.ply", cloud_in4, indices_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in4));
    ASSERT_TRUE(indices_in2 == indices_out);
}

TEST_F(YarpCloudUtilsTest, PLY_XYZI)
{
    using point_t = yarp::sig::DataXYZI;

    yarp::sig::PointCloud<point_t> cloud_out, cloud_in1, cloud_in2, cloud_in3, cloud_in4;
    yarp::sig::VectorOf<int> indices_out, indices_in1, indices_in2;

    populateIndices(indices_out, n_faces, std::rand);
    ASSERT_EQ(indices_out.size(), n_faces * 3);

    populateCloud(cloud_out, n_points, []() -> point_t { return {{rnf(), rnf(), rnf()}, rnf()}; });
    ASSERT_EQ(cloud_out.size(), n_points);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyzi_cloud_binary.ply", cloud_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyzi_cloud_binary.ply", cloud_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in1));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyzi_cloud_ascii.ply", cloud_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyzi_cloud_ascii.ply", cloud_in2));
    ASSERT_TRUE(cloud_out.toString() == cloud_in2.toString());

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyzi_mesh_binary.ply", cloud_out, indices_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyzi_mesh_binary.ply", cloud_in3, indices_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in3));
    ASSERT_TRUE(indices_in1 == indices_out);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyzi_mesh_ascii.ply", cloud_out, indices_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyzi_mesh_ascii.ply", cloud_in4, indices_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in4));
    ASSERT_TRUE(indices_in2 == indices_out);
}

TEST_F(YarpCloudUtilsTest, PLY_InterestPoint_XYZ)
{
    using point_t = yarp::sig::DataInterestPointXYZ;

    yarp::sig::PointCloud<point_t> cloud_out, cloud_in1, cloud_in2, cloud_in3, cloud_in4;
    yarp::sig::VectorOf<int> indices_out, indices_in1, indices_in2;

    populateIndices(indices_out, n_faces, std::rand);
    ASSERT_EQ(indices_out.size(), n_faces * 3);

    populateCloud(cloud_out, n_points, []() -> point_t { return {{rnf(), rnf(), rnf()}, rnf()}; });
    ASSERT_EQ(cloud_out.size(), n_points);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "interest_cloud_binary.ply", cloud_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "interest_cloud_binary.ply", cloud_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in1));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "interest_cloud_ascii.ply", cloud_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "interest_cloud_ascii.ply", cloud_in2));
    ASSERT_TRUE(cloud_out.toString() == cloud_in2.toString());

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "interest_mesh_binary.ply", cloud_out, indices_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "interest_mesh_binary.ply", cloud_in3, indices_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in3));
    ASSERT_TRUE(indices_in1 == indices_out);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "interest_mesh_ascii.ply", cloud_out, indices_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "interest_mesh_ascii.ply", cloud_in4, indices_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in4));
    ASSERT_TRUE(indices_in2 == indices_out);
}

TEST_F(YarpCloudUtilsTest, PLY_XYZ_Normal)
{
    using point_t = yarp::sig::DataXYZNormal;

    yarp::sig::PointCloud<point_t> cloud_out, cloud_in1, cloud_in2, cloud_in3, cloud_in4;
    yarp::sig::VectorOf<int> indices_out, indices_in1, indices_in2;

    populateIndices(indices_out, n_faces, std::rand);
    ASSERT_EQ(indices_out.size(), n_faces * 3);

    populateCloud(cloud_out, n_points, []() -> point_t { return {{rnf(), rnf(), rnf()}, {rnf(), rnf(), rnf()}, rnf()}; });
    ASSERT_EQ(cloud_out.size(), n_points);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_normal_cloud_binary.ply", cloud_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_normal_cloud_binary.ply", cloud_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in1));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_normal_cloud_ascii.ply", cloud_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_normal_cloud_ascii.ply", cloud_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in2));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_normal_mesh_binary.ply", cloud_out, indices_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_normal_mesh_binary.ply", cloud_in3, indices_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in3));
    ASSERT_TRUE(indices_in1 == indices_out);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_normal_mesh_ascii.ply", cloud_out, indices_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_normal_mesh_ascii.ply", cloud_in4, indices_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in4));
    ASSERT_TRUE(indices_in2 == indices_out);
}

TEST_F(YarpCloudUtilsTest, PLY_XYZ_Normal_RGBA)
{
    using point_t = yarp::sig::DataXYZNormalRGBA;

    yarp::sig::PointCloud<point_t> cloud_out, cloud_in1, cloud_in2, cloud_in3, cloud_in4;
    yarp::sig::VectorOf<int> indices_out, indices_in1, indices_in2;

    populateIndices(indices_out, n_faces, std::rand);
    ASSERT_EQ(indices_out.size(), n_faces * 3);

    populateCloud(cloud_out, n_points, []() -> point_t { return {{rnf(), rnf(), rnf()}, {rnf(), rnf(), rnf()}, {rnu(), rnu(), rnu(), rnu(), rnf()}}; });
    ASSERT_EQ(cloud_out.size(), n_points);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_normal_rgba_cloud_binary.ply", cloud_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_normal_rgba_cloud_binary.ply", cloud_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in1));

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_normal_rgba_cloud_ascii.ply", cloud_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_normal_rgba_cloud_ascii.ply", cloud_in2));
    ASSERT_TRUE(cloud_out.toString() == cloud_in2.toString());

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_normal_rgba_mesh_binary.ply", cloud_out, indices_out, true));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_normal_rgba_mesh_binary.ply", cloud_in3, indices_in1));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in3));
    ASSERT_TRUE(indices_in1 == indices_out);

    ASSERT_TRUE(YarpCloudUtils::savePLY(path + "xyz_normal_rgba_mesh_ascii.ply", cloud_out, indices_out, false));
    ASSERT_TRUE(YarpCloudUtils::loadPLY(path + "xyz_normal_rgba_mesh_ascii.ply", cloud_in4, indices_in2));
    ASSERT_TRUE(testEquality(cloud_out, cloud_in4));
    ASSERT_TRUE(indices_in2 == indices_out);
}

TEST_F(YarpCloudUtilsTest, meshFromCloud)
{
    yarp::sig::PointCloudXYZ points;
    populateCloud(points, 100, [] { return yarp::sig::DataXYZ {rnf(), rnf(), rnf()}; });

    yarp::sig::PointCloudXY points_xy(points), vertices_xy;
    yarp::sig::PointCloudXYZ points_xyz(points), vertices_xyz;
    yarp::sig::PointCloudNormal points_normal(points), vertices_normal;
    yarp::sig::PointCloudXYZRGBA points_xyz_rgba(points), vertices_xyz_rgba;
    yarp::sig::PointCloudXYZI points_xyzi(points), vertices_xyzi;
    yarp::sig::PointCloudInterestPointXYZ points_interest(points), vertices_interest;
    yarp::sig::PointCloudXYZNormal points_xyz_normal(points), vertices_xyz_normal;
    yarp::sig::PointCloudXYZNormalRGBA points_xyz_normal_rgba(points), vertices_xyz_normal_rgba;

    yarp::sig::VectorOf<int> indices;

    yarp::sig::VectorOf<yarp::os::Property> options {
        {{"algorithm", yarp::os::Value("ConvexHull")}},
        {{"algorithm", yarp::os::Value("SimplificationRemoveUnusedVertices")}}
    };

    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xy, vertices_xy, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xy, vertices_xyz, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xy, vertices_normal, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xy, vertices_xyz_rgba, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xy, vertices_xyzi, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xy, vertices_interest, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xy, vertices_xyz_normal, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xy, vertices_xyz_normal_rgba, indices, options));

    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_xy, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_xyz, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_xyz_rgba, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_xyzi, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_interest, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_xyz_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_xyz_normal_rgba, indices, options));

    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_normal, vertices_xy, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_normal, vertices_xyz, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_normal, vertices_normal, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_normal, vertices_xyz_rgba, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_normal, vertices_xyzi, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_normal, vertices_interest, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_normal, vertices_xyz_normal, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_normal, vertices_xyz_normal_rgba, indices, options));

    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz_rgba, vertices_xy, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_rgba, vertices_xyz, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz_rgba, vertices_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_rgba, vertices_xyz_rgba, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_rgba, vertices_xyzi, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_rgba, vertices_interest, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_rgba, vertices_xyz_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_rgba, vertices_xyz_normal_rgba, indices, options));

    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyzi, vertices_xy, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyzi, vertices_xyz, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyzi, vertices_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyzi, vertices_xyz_rgba, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyzi, vertices_xyzi, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyzi, vertices_interest, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyzi, vertices_xyz_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyzi, vertices_xyz_normal_rgba, indices, options));

    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_interest, vertices_xy, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_interest, vertices_xyz, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_interest, vertices_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_interest, vertices_xyz_rgba, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_interest, vertices_xyzi, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_interest, vertices_interest, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_interest, vertices_xyz_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_interest, vertices_xyz_normal_rgba, indices, options));

    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz_normal, vertices_xy, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal, vertices_xyz, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz_normal, vertices_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal, vertices_xyz_rgba, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal, vertices_xyzi, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal, vertices_interest, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal, vertices_xyz_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal, vertices_xyz_normal_rgba, indices, options));

    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz_normal_rgba, vertices_xy, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal_rgba, vertices_xyz, indices, options));
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz_normal_rgba, vertices_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal_rgba, vertices_xyz_rgba, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal_rgba, vertices_xyzi, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal_rgba, vertices_interest, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal_rgba, vertices_xyz_normal, indices, options));
    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points_xyz_normal_rgba, vertices_xyz_normal_rgba, indices, options));

    yarp::sig::VectorOf<yarp::os::Property> empty {};
    _ASSERT_FALSE(YarpCloudUtils::meshFromCloud(points_xyz, vertices_xyz, indices, empty));
}

TEST_F(YarpCloudUtilsTest, processCloud)
{
    yarp::sig::PointCloudXYZ points;
    populateCloud(points, 100, [] { return yarp::sig::DataXYZ {rnf(), rnf(), rnf()}; });

    yarp::sig::PointCloudXY in_xy(points), out_xy;
    yarp::sig::PointCloudXYZ in_xyz(points), out_xyz;
    yarp::sig::PointCloudNormal in_normal(points), out_normal;
    yarp::sig::PointCloudXYZRGBA in_xyz_rgba(points), out_xyz_rgba;
    yarp::sig::PointCloudXYZI in_xyzi(points), out_xyzi;
    yarp::sig::PointCloudInterestPointXYZ in_interest(points), out_interest;
    yarp::sig::PointCloudXYZNormal in_xyz_normal(points), out_xyz_normal;
    yarp::sig::PointCloudXYZNormalRGBA in_xyz_normal_rgba(points), out_xyz_normal_rgba;

    yarp::sig::VectorOf<yarp::os::Property> options {
        {{"algorithm", yarp::os::Value("ApproximateVoxelGrid")}, {"leafSize", yarp::os::Value(0.5f)}}
    };

    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xy, out_xy, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xy, out_xyz, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xy, out_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xy, out_xyz_rgba, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xy, out_xyzi, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xy, out_interest, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xy, out_xyz_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xy, out_xyz_normal_rgba, options));

    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz, out_xy, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz, out_xyz, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz, out_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz, out_xyz_rgba, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz, out_xyzi, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz, out_interest, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz, out_xyz_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz, out_xyz_normal_rgba, options));

    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_normal, out_xy, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_normal, out_xyz, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_normal, out_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_normal, out_xyz_rgba, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_normal, out_xyzi, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_normal, out_interest, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_normal, out_xyz_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_normal, out_xyz_normal_rgba, options));

    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_rgba, out_xy, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz_rgba, out_xyz, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_rgba, out_normal, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz_rgba, out_xyz_rgba, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_rgba, out_xyzi, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_rgba, out_interest, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_rgba, out_xyz_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_rgba, out_xyz_normal_rgba, options));

    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyzi, out_xy, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyzi, out_xyz, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyzi, out_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyzi, out_xyz_rgba, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyzi, out_xyzi, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyzi, out_interest, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyzi, out_xyz_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyzi, out_xyz_normal_rgba, options));

    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_interest, out_xy, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_interest, out_xyz, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_interest, out_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_interest, out_xyz_rgba, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_interest, out_xyzi, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_interest, out_interest, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_interest, out_xyz_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_interest, out_xyz_normal_rgba, options));

    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal, out_xy, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz_normal, out_xyz, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal, out_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal, out_xyz_rgba, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal, out_xyzi, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal, out_interest, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz_normal, out_xyz_normal, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal, out_xyz_normal_rgba, options));

    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal_rgba, out_xy, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz_normal_rgba, out_xyz, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal_rgba, out_normal, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz_normal_rgba, out_xyz_rgba, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal_rgba, out_xyzi, options));
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz_normal_rgba, out_interest, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz_normal_rgba, out_xyz_normal, options));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(in_xyz_normal_rgba, out_xyz_normal_rgba, options));

    yarp::sig::VectorOf<yarp::os::Property> empty {};
    _ASSERT_FALSE(YarpCloudUtils::processCloud(in_xyz, out_xyz, empty));
}

TEST_F(YarpCloudUtilsTest, pipelineFromConfig)
{
    yarp::sig::PointCloudXYZ points, vertices, transformed;
    populateCloud(points, 100, [] { return yarp::sig::DataXYZ {rnf(), rnf(), rnf()}; });

    yarp::sig::VectorOf<int> indices;
    std::string collection = "test";

    yarp::os::Property meshConfig("(test surface simplify)");
    meshConfig.addGroup("surface") = {{"algorithm", yarp::os::Value("ConvexHull")}};
    meshConfig.addGroup("simplify") = {{"algorithm", yarp::os::Value("SimplificationRemoveUnusedVertices")}};

    yarp::os::Property cloudConfig("(test downsample)");
    cloudConfig.addGroup("downsample") = {
        {"algorithm", yarp::os::Value("ApproximateVoxelGrid")},
        {"leafSize", yarp::os::Value(0.5f)}
    };

    _ASSERT_TRUE(YarpCloudUtils::meshFromCloud(points, vertices, indices, meshConfig, collection));
    _ASSERT_TRUE(YarpCloudUtils::processCloud(points, transformed, cloudConfig, collection));
}

} // namespace roboticslab::test
