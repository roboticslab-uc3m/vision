#include "YarpCloudUtils.hpp"

#include <utility> // std::move

#include <yarp/pcl/Pcl.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

namespace
{
    template <typename T>
    struct pcl_type_from_yarp;

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXY>
    { typedef pcl::PointXY type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZ>
    { typedef pcl::PointXYZ type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataNormal>
    { typedef pcl::Normal type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZRGBA>
    { typedef pcl::PointXYZRGBA type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZI>
    { typedef pcl::PointXYZI type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataInterestPointXYZ>
    { typedef pcl::InterestPoint type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZNormal>
    { typedef pcl::PointNormal type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZNormalRGBA>
    { typedef pcl::PointXYZRGBNormal type; };

    auto verticesToPCL(const yarp::sig::VectorOf<int> & vertices)
    {
        const std::size_t n_triangles = vertices.size() / 3;
        std::vector<pcl::Vertices> out;
        out.reserve(n_triangles);

        for (auto i = 0; i < n_triangles; i += 3)
        {
            pcl::Vertices triangles;

            triangles.vertices = {
                static_cast<decltype(triangles.vertices)::value_type>(vertices[3 * i]),
                static_cast<decltype(triangles.vertices)::value_type>(vertices[3 * i + 1]),
                static_cast<decltype(triangles.vertices)::value_type>(vertices[3 * i + 2])
            };

            out.emplace_back(std::move(triangles));
        }

        return out;
    }

    auto verticesFromPCL(const std::vector<pcl::Vertices> & triangles)
    {
        yarp::sig::VectorOf<int> out;
        out.reserve(triangles.size() * 3);

        for (auto i = 0; i < triangles.size(); i++)
        {
            const auto & vertices = triangles[i].vertices;
            out.emplace_back(vertices[0]);
            out.emplace_back(vertices[1]);
            out.emplace_back(vertices[2]);
        }

        return out;
    }
}

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, bool isBinary)
{
    using pcl_type = typename pcl_type_from_yarp<T>::type;
    pcl::PointCloud<pcl_type> pcl;
    return yarp::pcl::toPCL(cloud, pcl) && pcl::io::savePLYFile(filename, pcl, isBinary) == 0;
}

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
{
    using pcl_type = typename pcl_type_from_yarp<T>::type;
    pcl::PointCloud<pcl_type> pcl;
    yarp::pcl::toPCL(cloud, pcl);
    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(pcl, mesh.cloud);
    mesh.polygons = verticesToPCL(vertices);
    return pcl::io::savePLYFile(filename, mesh);
}

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud)
{
    using pcl_type = typename pcl_type_from_yarp<T>::type;
    pcl::PointCloud<pcl_type> pcl;
    return pcl::io::loadPLYFile(filename, pcl) == 0 && yarp::pcl::fromPCL(pcl, cloud);
}

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud, yarp::sig::VectorOf<int> & vertices)
{
    using pcl_type = typename pcl_type_from_yarp<T>::type;
    pcl::PolygonMesh mesh;

    if (pcl::io::loadPLYFile(filename, mesh) != 0)
    {
        return false;
    }

    pcl::PointCloud<pcl_type> pcl;
    pcl::fromPCLPointCloud2(mesh.cloud, pcl);
    vertices = verticesFromPCL(mesh.polygons);
    return yarp::pcl::fromPCL(pcl, cloud);
}

} // namespace YarpCloudUtils

} // namespace roboticslab

// explicit instantiations
#include "YarpCloudUtils-inst.hpp"
