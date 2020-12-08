#include "gtest/gtest.h"

#include <cmath>
#include <cstdlib>
#include <string>

#include "YarpCloudUtils.hpp"

#ifndef PLY_PATH
#error "missing compile definition PLY_PATH"
#endif

namespace roboticslab
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
void populateVertices(yarp::sig::VectorOf<int> & vertices, int faces, Fn && generator)
{
    vertices.reserve(faces * 3);

    for (auto i = 0; i < faces * 3; i++)
    {
        vertices.push_back(generator());
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

    populateVertices(indices_out, n_faces, std::rand);
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

    populateVertices(indices_out, n_faces, std::rand);
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

    populateVertices(indices_out, n_faces, std::rand);
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

    populateVertices(indices_out, n_faces, std::rand);
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

    populateVertices(indices_out, n_faces, std::rand);
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

    populateVertices(indices_out, n_faces, std::rand);
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

    populateVertices(indices_out, n_faces, std::rand);
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

    populateVertices(indices_out, n_faces, std::rand);
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

} // namespace roboticslab
