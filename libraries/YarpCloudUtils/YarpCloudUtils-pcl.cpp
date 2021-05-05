// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCloudUtils.hpp"

#include <yarp/os/LogStream.h>
#ifdef HAVE_PCL
#include <yarp/pcl/Pcl.h>
#include <yarp/conf/version.h>

#include <cstdint>

#include <exception>
#include <stdexcept>
#include <utility>
#include <type_traits>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h> // pcl::copyPointCloud

#include "YarpCloudUtils-pcl.hpp"
#include "YarpCloudUtils-pcl-traits.hpp"
#include "YarpCloudUtils-pcl-impl.hpp"

namespace
{
    // https://gist.github.com/Lee-R/3839813

    constexpr std::uint32_t fnv1a_32(const char * s, std::size_t count)
    {
        return ((count ? fnv1a_32(s, count - 1) : 2166136261u) ^ s[count]) * 16777619u;
    }

    constexpr std::uint32_t operator"" _hash(const char * s, std::size_t count)
    {
        return fnv1a_32(s, count);
    }

    std::uint32_t makeHash(const std::string & s)
    {
        return fnv1a_32(s.c_str(), s.size());
    }

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

    template <typename T>
    void processStep(const cloud_container & prev, cloud_container & curr, const yarp::os::Searchable & options)
    {
        using any_xyz_t = typename pcl_decay<T, pcl_all_xyz_types_tag>::type;
        using xyz_rgb_t = typename pcl_decay<T, pcl_xyz_rgb_types_tag>::type;
        using rgb_t = typename pcl_decay<T, pcl_rgb_types_tag>::type;
        using xyzi_t = typename pcl_decay<T, pcl_xyzi_types_tag>::type;
        using normal_t = typename pcl_decay<T, pcl_normal_types_tag>::type;

        if (!options.check("algorithm"))
        {
            throw std::invalid_argument("missing algorithm parameter");
        }

        yDebug() << "step:" << options.toString();

        auto algorithm = options.find("algorithm").asString();

        switch (makeHash(algorithm))
        {
        case "transformPointCloud"_hash:
            doTransformPointCloud<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "transformPointCloudWithNormals"_hash:
            doTransformPointCloudWithNormals<normal_t>(prev.getCloud<normal_t>(), curr.setCloud<normal_t>(), options);
            break;
        case "ApproximateVoxelGrid"_hash:
            doApproximateVoxelGrid<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "BilateralFilter"_hash:
            doBilateralFilter<xyzi_t>(prev.getCloud<xyzi_t>(), curr.setCloud<xyzi_t>(), options);
            break;
        case "BilateralUpsampling"_hash:
            doBilateralUpsampling<rgb_t>(prev.getCloud<rgb_t>(), curr.setCloud<rgb_t>(), options);
            break;
        case "ConcaveHull"_hash:
            doConcaveHull<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setMesh(), options);
            break;
        case "ConvexHull"_hash:
            doConvexHull<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setMesh(), options);
            break;
        case "CropBox"_hash:
            doCropBox<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "FastBilateralFilter"_hash:
            doFastBilateralFilter<xyz_rgb_t>(prev.getCloud<xyz_rgb_t>(), curr.setCloud<xyz_rgb_t>(), options);
            break;
        case "FastBilateralFilterOMP"_hash:
            doFastBilateralFilterOMP<xyz_rgb_t>(prev.getCloud<xyz_rgb_t>(), curr.setCloud<xyz_rgb_t>(), options);
            break;
        case "GridMinimum"_hash:
            doGridMinimum<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "GreedyProjectionTriangulation"_hash:
            doGreedyProjectionTriangulation<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "GridProjection"_hash:
            doGridProjection<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "LocalMaximum"_hash:
            doLocalMaximum<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "MarchingCubesHoppe"_hash:
            doMarchingCubesHoppe<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "MarchingCubesRBF"_hash:
            doMarchingCubesRBF<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "MedianFilter"_hash:
            doMedianFilter<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "MeshQuadricDecimationVTK"_hash:
            doMeshQuadricDecimationVTK(prev.getMesh(), curr.setMesh(), options);
            break;
        case "MeshSmoothingLaplacianVTK"_hash:
            doMeshSmoothingLaplacianVTK(prev.getMesh(), curr.setMesh(), options);
            break;
        case "MeshSmoothingWindowedSincVTK"_hash:
            doMeshSmoothingWindowedSincVTK(prev.getMesh(), curr.setMesh(), options);
            break;
        case "MeshSubdivisionVTK"_hash:
            doMeshSubdivisionVTK(prev.getMesh(), curr.setMesh(), options);
            break;
#if PCL_VERSION_COMPARE(>=, 1, 9, 0)
        case "MovingLeastSquares"_hash:
            if (options.check("computeNormals"), yarp::os::Value(false).asBool())
                doMovingLeastSquares<any_xyz_t, normal_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<normal_t>(), options);
            else
                doMovingLeastSquares<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
#endif
        case "NormalEstimation"_hash:
            pcl::copyPointCloud(*prev.getCloud<any_xyz_t>(), *curr.setCloud<normal_t>());
            doNormalEstimation<any_xyz_t, normal_t>(prev.getCloud<any_xyz_t>(), curr.useCloud<normal_t>(), options);
            break;
        case "NormalEstimationOMP"_hash:
            pcl::copyPointCloud(*prev.getCloud<any_xyz_t>(), *curr.setCloud<normal_t>());
            doNormalEstimationOMP<any_xyz_t, normal_t>(prev.getCloud<any_xyz_t>(), curr.useCloud<normal_t>(), options);
            break;
        case "OrganizedFastMesh"_hash:
            doOrganizedFastMesh<xyz_rgb_t>(prev.getCloud<xyz_rgb_t>(), curr.setMesh(), options);
            break;
        case "PassThrough"_hash:
            doPassThrough<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "Poisson"_hash:
            doPoisson<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "RadiusOutlierRemoval"_hash:
            doRadiusOutlierRemoval<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "RandomSample"_hash:
            doRandomSample<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "SamplingSurfaceNormal"_hash:
            doSamplingSurfaceNormal<normal_t>(prev.getCloud<normal_t>(), curr.setCloud<normal_t>(), options);
            break;
        case "ShadowPoints"_hash:
            doShadowPoints<normal_t>(prev.getCloud<normal_t>(), curr.setCloud<normal_t>(), options);
            break;
        case "SimplificationRemoveUnusedVertices"_hash:
            doSimplificationRemoveUnusedVertices(prev.getMesh(), curr.setMesh(), options);
            break;
        case "StatisticalOutlierRemoval"_hash:
            doStatisticalOutlierRemoval<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "UniformSampling"_hash:
            doUniformSampling<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "VoxelGrid"_hash:
            doVoxelGrid<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        default:
            throw std::invalid_argument("unsupported algorithm: " + algorithm);
        }
    }

    template <typename T, std::enable_if_t<is_unsupported_type<T>, bool> = true>
    void meshFromCloudPCL(const typename pcl::PointCloud<T>::Ptr &, pcl::PolygonMesh::ConstPtr &, const yarp::sig::VectorOf<yarp::os::Property> &)
    {
        throw std::invalid_argument("unsupported point type"); // don't remove this
    }

    template <typename T, std::enable_if_t<!is_unsupported_type<T>, bool> = true>
    void meshFromCloudPCL(const typename pcl::PointCloud<T>::Ptr & cloud, pcl::PolygonMesh::ConstPtr & mesh, const yarp::sig::VectorOf<yarp::os::Property> & options)
    {
        cloud_container data;
        data.setCloud<T>() = cloud;

#if YARP_VERSION_MINOR >= 4
        for (const auto & step : options)
        {
            cloud_container temp;
            processStep<T>(data, temp, step);
            data = std::move(temp);
        }
#else
        for (auto i = 0; i < options.size(); i++)
        {
            cloud_container temp;
            processStep<T>(data, temp, options[i]);
            data = std::move(temp);
        }
#endif

        mesh = data.getMesh();
    }

    template <typename T1, typename T2, std::enable_if_t<is_unsupported_type<T1> || is_unsupported_type<T2>, bool> = true>
    void processCloudPCL(const typename pcl::PointCloud<T1>::Ptr &, typename pcl::PointCloud<T2>::ConstPtr &, const yarp::sig::VectorOf<yarp::os::Property> &)
    {
        throw std::invalid_argument("unsupported point type"); // don't remove this
    }

    template <typename T1, typename T2, std::enable_if_t<!is_unsupported_type<T1> && !is_unsupported_type<T2>, bool> = true>
    void processCloudPCL(const typename pcl::PointCloud<T1>::Ptr & in, typename pcl::PointCloud<T2>::ConstPtr & out, const yarp::sig::VectorOf<yarp::os::Property> & options)
    {
        cloud_container data;
        data.setCloud<T1>() = in;

#if YARP_VERSION_MINOR >= 4
        for (const auto & step : options)
        {
            cloud_container temp;
            processStep<T1>(data, temp, step);
            data = std::move(temp);
        }
#else
        for (auto i = 0; i < options.size(); i++)
        {
            cloud_container temp;
            processStep<T1>(data, temp, options[i]);
            data = std::move(temp);
        }
#endif

        out = data.getCloud<T2>();
    }
}
#endif

namespace
{
    auto makeFromConfig(const yarp::os::Searchable & config, const std::string & collection)
    {
        yarp::sig::VectorOf<yarp::os::Property> options;
        const auto & pipeline = config.findGroup(collection);

        if (!pipeline.isNull())
        {
            auto groups = pipeline.tail();

            for (auto i = 0; i < groups.size(); i++)
            {
                auto groupName = groups.get(i).asString();
                const auto & group = config.findGroup(groupName);

                if (!group.isNull())
                {
                    auto groupConfig = group.tail();
                    options.push_back(groupConfig.toString().c_str());
                }
                else
                {
                    yWarning() << "group not found:" << groupName;
                }
            }
        }
        else
        {
            yWarning() << "collection not found:" << collection;
        }

        return options;
    }
}

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T1, typename T2>
bool meshFromCloud(const yarp::sig::PointCloud<T1> & cloud,
                   yarp::sig::PointCloud<T2> & meshPoints,
                   yarp::sig::VectorOf<int> & meshIndices,
                   const yarp::sig::VectorOf<yarp::os::Property> & options)
{
#ifdef HAVE_PCL
    using pcl_input_type = typename pcl_type_from_yarp<T1>::type;
    using pcl_output_type = typename pcl_type_from_yarp<T2>::type;

    // This variable and its following checks aim to override compiler optimizations, thus forcing
    // the compiler/linker to instantiate this signature of meshFromCloud for unsupported point types.
    // Otherwise, plain is_unsupported_type checks would make GCC strip several symbols from the .so.

    volatile auto dummy = true;

    if (is_unsupported_type<pcl_input_type> && dummy)
    {
        yError() << "unsupported input point type" << pcl_descriptor<pcl_input_type>::name;
        return false;
    }

    if (is_unsupported_type<pcl_output_type> && dummy)
    {
        yError() << "unsupported output point type" << pcl_descriptor<pcl_output_type>::name;
        return false;
    }

    if (options.size() == 0)
    {
        yError() << "empty configuration";
        return false;
    }

    typename pcl::PointCloud<pcl_input_type>::Ptr pclCloud(new pcl::PointCloud<pcl_input_type>());

    // Convert YARP cloud to PCL cloud.
    yarp::pcl::toPCL(cloud, *pclCloud);

    // Perform surface reconstruction.
    pcl::PolygonMesh::ConstPtr pclMesh;

    try
    {
        meshFromCloudPCL<pcl_input_type>(pclCloud, pclMesh, options);
    }
    catch (const std::exception & e)
    {
        yError() << "meshFromCloudPCL:" << e.what();
        return false;
    }

    // Extract point cloud of vertices from mesh.
    typename pcl::PointCloud<pcl_output_type>::Ptr pclMeshPoints(new pcl::PointCloud<pcl_output_type>());
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

template <typename T1, typename T2>
bool meshFromCloud(const yarp::sig::PointCloud<T1> & cloud,
                   yarp::sig::PointCloud<T2> & meshPoints,
                   yarp::sig::VectorOf<int> & meshIndices,
                   const yarp::os::Searchable & config,
                   const std::string & collection)
{
    return meshFromCloud(cloud, meshPoints, meshIndices, makeFromConfig(config, collection));
}

template <typename T1, typename T2>
bool processCloud(const yarp::sig::PointCloud<T1> & in,
                  yarp::sig::PointCloud<T2> & out,
                  const yarp::sig::VectorOf<yarp::os::Property> & options)
{
#ifdef HAVE_PCL
    using pcl_input_type = typename pcl_type_from_yarp<T1>::type;
    using pcl_output_type = typename pcl_type_from_yarp<T2>::type;

    // See analogous comment in meshFromCloud.
    volatile auto dummy = true;

    if (is_unsupported_type<pcl_input_type> && dummy)
    {
        yError() << "unsupported input point type" << pcl_descriptor<pcl_input_type>::name;
        return false;
    }

    if (is_unsupported_type<pcl_output_type> && dummy)
    {
        yError() << "unsupported output point type" << pcl_descriptor<pcl_output_type>::name;
        return false;
    }

    if (options.size() == 0)
    {
        yError() << "empty configuration";
        return false;
    }

    typename pcl::PointCloud<pcl_input_type>::Ptr pclCloudIn(new pcl::PointCloud<pcl_input_type>());

    // Convert YARP cloud to PCL cloud.
    yarp::pcl::toPCL(in, *pclCloudIn);

    // Perform cloud processing.
    typename pcl::PointCloud<pcl_output_type>::ConstPtr pclCloudOut;

    try
    {
        processCloudPCL<pcl_input_type, pcl_output_type>(pclCloudIn, pclCloudOut, options);
    }
    catch (const std::exception & e)
    {
        yError() << "processCloud:" << e.what();
        return false;
    }

    // Convert PCL cloud to YARP cloud.
    yarp::pcl::fromPCL(*pclCloudOut, out);

    return true;
#else
    yError() << "processCloud compiled with no PCL support";
    return false;
#endif
}

template <typename T1, typename T2>
bool processCloud(const yarp::sig::PointCloud<T1> & in,
                  yarp::sig::PointCloud<T2> & out,
                  const yarp::os::Searchable & config,
                  const std::string & collection)
{
    return processCloud(in, out, makeFromConfig(config, collection));
}

} // namespace YarpCloudUtils

} // namespace roboticslab

// explicit instantiations
#include "YarpCloudUtils-pcl-inst.hpp"
