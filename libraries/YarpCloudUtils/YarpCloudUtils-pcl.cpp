#include "YarpCloudUtils.hpp"

#include <yarp/os/LogStream.h>
#ifdef HAVE_PCL
#include <yarp/pcl/Pcl.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
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

    bool meshFromCloudPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PolygonMesh::Ptr & mesh, const yarp::os::Searchable & options)
    {
        // Downsample so that further computations are actually feasible.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setInputCloud(cloud);
        grid.setLeafSize(0.01f, 0.01f, 0.01f);
        grid.filter(*cloud2);

        // Estimate normals.
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> estimator;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud2);
        estimator.setNumberOfThreads(0);
        estimator.setInputCloud(cloud2);
        estimator.setSearchMethod(tree);
        estimator.setKSearch(40);
        estimator.compute(*normals);

        // Concatenate point clouds.
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
        pcl::concatenateFields(*cloud2, *normals, *cloudWithNormals);

        // Reconstruct triangle mesh.
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>());
        tree2->setInputCloud(cloudWithNormals);

        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setInputCloud(cloudWithNormals);
        poisson.setSearchMethod(tree2);
        poisson.reconstruct(*mesh);

        return true;
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
