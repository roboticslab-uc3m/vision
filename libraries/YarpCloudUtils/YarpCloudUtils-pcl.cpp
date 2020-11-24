#include "YarpCloudUtils.hpp"

#include <yarp/os/LogStream.h>
#ifdef HAVE_PCL
#include <yarp/pcl/Pcl.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/poisson.h>

namespace
{
    auto indicesFromPCL(const std::vector<pcl::Vertices> & triangles)
    {
        yarp::sig::VectorOf<int> out(triangles.size() * 3);

        for (auto i = 0; i < triangles.size(); i++)
        {
            const auto & indices = triangles[i].vertices;
            out[3 * i] = indices[0];
            out[3 * i + 1] = indices[1];
            out[3 * i + 2] = indices[2];
        }

        return out;
    }

    void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr & in, const pcl::PointCloud<pcl::PointXYZ>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto leafSize = options.check("filterLeafSize", yarp::os::Value(0.01f)).asFloat32();
        auto leafSizeX = options.check("filterLeafSizeX", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeY = options.check("filterLeafSizeY", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeZ = options.check("filterLeafSizeZ", yarp::os::Value(leafSize)).asFloat32();
        auto limitMax = options.check("filterLimitMax", yarp::os::Value(FLT_MAX)).asFloat64();
        auto limitMin = options.check("filterLimitMin", yarp::os::Value(-FLT_MAX)).asFloat64();
        auto limitsNegative = options.check("filterLimitsNegative", yarp::os::Value(false)).asBool();
        auto minPointsPerVoxel = options.check("filterMinPointsPerVoxel", yarp::os::Value(0)).asInt32();

        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setDownsampleAllData(false);
        grid.setFilterLimits(limitMin, limitMax);
        grid.setFilterLimitsNegative(limitsNegative);
        grid.setInputCloud(in);
        grid.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
        grid.setMinimumPointsNumberPerVoxel(minPointsPerVoxel);
        grid.filter(*out);
    }

    void estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr & in, const pcl::PointCloud<pcl::Normal>::Ptr & out, const yarp::os::Searchable & options)
    {
        // Either radius search or nearest K search; one of these params must be zero.
        auto k = options.check("estimatorK", yarp::os::Value(40)).asInt32();
        auto radius = options.check("estimatorRadius", yarp::os::Value(0.0)).asFloat64();

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr estimator;

        if (options.check("estimatorUseOMP", yarp::os::Value(true)).asBool())
        {
            auto threads = options.check("estimatorThreads", yarp::os::Value(0)).asInt32();
            auto * omp = new pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>();
            omp->setNumberOfThreads(threads);
            estimator.reset(omp);
        }
        else
        {
            estimator.reset(new pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>());
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(in);
        estimator->setInputCloud(in);
        estimator->setKSearch(k);
        estimator->setRadiusSearch(radius);
        estimator->setSearchMethod(tree);
        estimator->compute(*out);
    }

    void reconstruct(const pcl::PointCloud<pcl::PointNormal>::Ptr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        const auto fallback = "poisson";
        auto method = options.check("surfaceMethod", yarp::os::Value(fallback)).asString();

        pcl::PCLSurfaceBase<pcl::PointNormal>::Ptr surface;

        if (method == "convex")
        {
            auto * convex = new pcl::ConvexHull<pcl::PointNormal>();
            surface.reset(convex);
        }
        else if (method == "concave")
        {
            auto * concave = new pcl::ConcaveHull<pcl::PointNormal>();
            surface.reset(concave);
        }
        else if (method == "gpt")
        {
            auto * gpt = new pcl::GreedyProjectionTriangulation<pcl::PointNormal>();
            surface.reset(gpt);
        }
        else if (method == "gp")
        {
            auto * gp = new pcl::GridProjection<pcl::PointNormal>();
            surface.reset(gp);
        }
        else if (method == "poisson")
        {
            auto * poisson = new pcl::Poisson<pcl::PointNormal>();
            surface.reset(poisson);
        }
        else
        {
            yWarning() << "unrecognized surface method:" << method;
            yInfo() << "falling back with default parameters to" << fallback;
        }

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
        tree->setInputCloud(in);
        surface->setInputCloud(in);
        surface->setSearchMethod(tree);
        surface->reconstruct(*out);
    }

    void meshFromCloudPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::PolygonMesh::Ptr & mesh, const yarp::os::Searchable & options)
    {
        // Downsample so that further computations are actually feasible.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
        downsample(cloud, cloud2, options);

        // Estimate normals.
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        estimateNormals(cloud2, normals, options);

        // Concatenate point clouds.
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
        pcl::concatenateFields(*cloud2, *normals, *cloudWithNormals);

        // Reconstruct triangle mesh.
        reconstruct(cloudWithNormals, mesh, options);
    }
}
#endif

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T>
bool meshFromCloud(const yarp::sig::PointCloud<T> & cloud, yarp::sig::PointCloudXYZ & meshPoints, yarp::sig::VectorOf<int> & meshIndices,
    const yarp::os::Searchable & options)
{
#ifdef HAVE_PCL
    // Get a XYZ point cloud.
    yarp::sig::PointCloudXYZ cloudXYZ(cloud);

    // Convert YARP cloud to PCL cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclXYZ(new pcl::PointCloud<pcl::PointXYZ>());
    yarp::pcl::toPCL(cloudXYZ, *pclXYZ);

    // Perform surface reconstruction.
    pcl::PolygonMesh::Ptr pclMesh(new pcl::PolygonMesh());
    meshFromCloudPCL(pclXYZ, pclMesh, options);

    // Extract point cloud of vertices from mesh.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclMeshPoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(pclMesh->cloud, *pclMeshPoints);

    // Convert PCL mesh to YARP cloud and vector of indices.
    yarp::pcl::fromPCL(*pclMeshPoints, meshPoints);
    meshIndices = indicesFromPCL(pclMesh->polygons);

    return true;
#else
    yError() << "meshFromCloud compiled with no PCL support";
    return false;
#endif
}

// explicit instantiations

template bool meshFromCloud(const yarp::sig::PointCloudXY &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &);
template bool meshFromCloud(const yarp::sig::PointCloudXYZ &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &);
template bool meshFromCloud(const yarp::sig::PointCloudNormal &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &);
template bool meshFromCloud(const yarp::sig::PointCloudXYZRGBA &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &);
template bool meshFromCloud(const yarp::sig::PointCloudXYZI &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &);
template bool meshFromCloud(const yarp::sig::PointCloudInterestPointXYZ &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &);
template bool meshFromCloud(const yarp::sig::PointCloudXYZNormal &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &);
template bool meshFromCloud(const yarp::sig::PointCloudXYZNormalRGBA &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &);

} // namespace YarpCloudUtils

} // namespace roboticslab
