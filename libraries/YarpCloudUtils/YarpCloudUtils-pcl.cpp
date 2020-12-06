// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCloudUtils.hpp"

#include <yarp/os/LogStream.h>
#ifdef HAVE_PCL
#include <yarp/pcl/Pcl.h>

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>
#include <cstdint>

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <type_traits>

#include <pcl/pcl_config.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>

#include "YarpCloudUtils-pcl-traits.hpp"

namespace
{
    // https://gist.github.com/Lee-R/3839813

    constexpr std::uint32_t fnv1a_32(const char * s, std::size_t count)
    {
        return ((count ? fnv1a_32(s, count - 1) : 2166136261u) ^ s[count]) * 16777619u;
    }

    std::uint32_t makeHash(const std::string & s)
    {
        return fnv1a_32(s.c_str(), s.size());
    }

    constexpr std::uint32_t operator"" _hash(const char * s, std::size_t count)
    {
        return fnv1a_32(s, count);
    }

    template <typename T1, typename T2, std::enable_if_t<!std::is_same<T1, T2>::value, bool> = true>
    auto initializeCloudPointer(const typename pcl::PointCloud<T1>::ConstPtr & in)
    {
        typename pcl::PointCloud<T2>::Ptr out(new pcl::PointCloud<T2>());
        pcl::copyPointCloud(*in, *out);
        return out;
    }

    template <typename T1, typename T2, std::enable_if_t<std::is_same<T1, T2>::value, bool> = true>
    auto initializeCloudPointer(const typename pcl::PointCloud<T1>::ConstPtr & in)
    {
        return in;
    }

    class cloud_container
    {
    public:
        template <typename T>
        typename pcl::PointCloud<T>::ConstPtr getCloud() const
        {
            if (xyz)
                return initializeCloudPointer<pcl::PointXYZ, T>(xyz);
            else if (xyz_rgb)
                return initializeCloudPointer<pcl::PointXYZRGB, T>(xyz_rgb);
            else if (xyzi)
                return initializeCloudPointer<pcl::PointXYZI, T>(xyzi);
            else if (xyz_interest)
                return initializeCloudPointer<pcl::InterestPoint, T>(xyz_interest);
            else if (xyz_normal)
                return initializeCloudPointer<pcl::PointNormal, T>(xyz_normal);
            else if (xyz_rgb_normal)
                return initializeCloudPointer<pcl::PointXYZRGBNormal, T>(xyz_rgb_normal);
            else if (xyzi_normal)
                return initializeCloudPointer<pcl::PointXYZINormal, T>(xyzi_normal);
            else
                throw std::runtime_error("cloud pointer was not initialized");
        }

        template <typename T>
        typename pcl::PointCloud<T>::Ptr & setCloud();

        template <typename T>
        const typename pcl::PointCloud<T>::Ptr & useCloud() const;

        const pcl::PolygonMesh::Ptr & getMesh() const
        {
            if (mesh)
                return mesh;
            else
                throw std::runtime_error("mesh pointer was not initialized");
        }

        pcl::PolygonMesh::Ptr & setMesh()
        { return mesh.reset(new pcl::PolygonMesh()), mesh; }

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb;
        pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi;
        pcl::PointCloud<pcl::InterestPoint>::Ptr xyz_interest;
        pcl::PointCloud<pcl::PointNormal>::Ptr xyz_normal;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr xyz_rgb_normal;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr xyzi_normal;
        pcl::PolygonMesh::Ptr mesh;
    };

    template <>
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_container::setCloud<pcl::PointXYZ>()
    { return xyz.reset(new pcl::PointCloud<pcl::PointXYZ>()), xyz; }

    template <>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_container::setCloud<pcl::PointXYZRGB>()
    { return xyz_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>()), xyz_rgb; }

    template <>
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_container::setCloud<pcl::PointXYZI>()
    { return xyzi.reset(new pcl::PointCloud<pcl::PointXYZI>()), xyzi; }

    template <>
    pcl::PointCloud<pcl::InterestPoint>::Ptr & cloud_container::setCloud<pcl::InterestPoint>()
    { return xyz_interest.reset(new pcl::PointCloud<pcl::InterestPoint>()), xyz_interest; }

    template <>
    pcl::PointCloud<pcl::PointNormal>::Ptr & cloud_container::setCloud<pcl::PointNormal>()
    { return xyz_normal.reset(new pcl::PointCloud<pcl::PointNormal>()), xyz_normal; }

    template <>
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud_container::setCloud<pcl::PointXYZRGBNormal>()
    { return xyz_rgb_normal.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>()), xyz_rgb_normal; }

    template <>
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud_container::setCloud<pcl::PointXYZINormal>()
    { return xyzi_normal.reset(new pcl::PointCloud<pcl::PointXYZINormal>()), xyzi_normal; }

    template <>
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_container::useCloud<pcl::PointXYZ>() const
    { return xyz; }

    template <>
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_container::useCloud<pcl::PointXYZRGB>() const
    { return xyz_rgb; }

    template <>
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_container::useCloud<pcl::PointXYZI>() const
    { return xyzi; }

    template <>
    const pcl::PointCloud<pcl::InterestPoint>::Ptr & cloud_container::useCloud<pcl::InterestPoint>() const
    { return xyz_interest; }

    template <>
    const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud_container::useCloud<pcl::PointNormal>() const
    { return xyz_normal; }

    template <>
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud_container::useCloud<pcl::PointXYZRGBNormal>() const
    { return xyz_rgb_normal; }

    template <>
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud_container::useCloud<pcl::PointXYZINormal>() const
    { return xyzi_normal; }

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
    void checkOutput(const typename pcl::PointCloud<T>::ConstPtr & cloud, const std::string & caller)
    {
        if (cloud->empty())
        {
            throw std::runtime_error(caller + " returned an empty cloud");
        }
    }

    void checkOutput(const pcl::PolygonMesh::ConstPtr & mesh, const std::string & caller)
    {
        if (mesh->cloud.data.empty() || mesh->polygons.empty())
        {
            throw std::runtime_error(caller + " returned an empty or incomplete mesh");
        }
    }

    template <typename T>
    void doApproximateVoxelGrid(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto downsampleAllData = options.check("downsampleAllData", yarp::os::Value(true)).asBool();
        auto leafSize = options.check("leafSize", yarp::os::Value(0.0f)).asFloat32();
        auto leafSizeX = options.check("leafSizeX", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeY = options.check("leafSizeY", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeZ = options.check("leafSizeZ", yarp::os::Value(leafSize)).asFloat32();

        pcl::ApproximateVoxelGrid<T> grid;
        grid.setDownsampleAllData(downsampleAllData);
        grid.setInputCloud(in);
        grid.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
        grid.filter(*out);

        checkOutput<T>(out, "ApproximateVoxelGrid");
    }

    template <typename T>
    void doConcaveHull(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto alpha = options.check("alpha", yarp::os::Value(0.0)).asFloat64();

        pcl::ConcaveHull<T> concave;
        concave.setAlpha(alpha);
        concave.setDimension(3);
        concave.setInputCloud(in);
        concave.setKeepInformation(true);
        concave.reconstruct(*out);

        checkOutput(out, "ConcaveHull");
    }

    template <typename T>
    void doConvexHull(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        pcl::ConvexHull<T> convex;
        convex.setDimension(3);
        convex.setInputCloud(in);
        convex.reconstruct(*out);

        checkOutput(out, "ConvexHull");
    }

    template <typename T>
    void doCropBox(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto maxX = options.check("maxX", yarp::os::Value(0.0f)).asFloat32();
        auto maxY = options.check("maxY", yarp::os::Value(0.0f)).asFloat32();
        auto maxZ = options.check("maxZ", yarp::os::Value(0.0f)).asFloat32();
        auto minX = options.check("minX", yarp::os::Value(0.0f)).asFloat32();
        auto minY = options.check("minY", yarp::os::Value(0.0f)).asFloat32();
        auto minZ = options.check("minZ", yarp::os::Value(0.0f)).asFloat32();
        auto negative = options.check("negative", yarp::os::Value(false)).asBool();
        auto rotationX = options.check("rotationX", yarp::os::Value(0.0f)).asFloat32();
        auto rotationY = options.check("rotationY", yarp::os::Value(0.0f)).asFloat32();
        auto rotationZ = options.check("rotationZ", yarp::os::Value(0.0f)).asFloat32();
        auto translationX = options.check("translationX", yarp::os::Value(0.0f)).asFloat32();
        auto translationY = options.check("translationY", yarp::os::Value(0.0f)).asFloat32();
        auto translationZ = options.check("translationZ", yarp::os::Value(0.0f)).asFloat32();

        pcl::CropBox<T> cropper;
        cropper.setInputCloud(in);
        cropper.setKeepOrganized(true); // don't remove points from the cloud, fill holes with NaNs
        cropper.setMax({maxX, maxY, maxZ, 1.0f});
        cropper.setMin({minX, minY, minZ, 1.0f});
        cropper.setNegative(negative);
        cropper.setRotation({rotationX, rotationY, rotationZ});
        cropper.setTranslation({translationX, translationY, translationZ});
        cropper.filter(*out);

        checkOutput<T>(out, "CropBox");
    }

    template <typename T>
    void doFastBilateralFilter(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        if (!in->isOrganized())
        {
            throw std::invalid_argument("input cloud must be organized (height > 1) for FastBilateralFilter");
        }

        auto sigmaR = options.check("sigmaR", yarp::os::Value(0.05f)).asFloat32();
        auto sigmaS = options.check("sigmaS", yarp::os::Value(15.0f)).asFloat32();

        pcl::FastBilateralFilter<T> fast;
        fast.setInputCloud(in);
        fast.setSigmaR(sigmaR);
        fast.setSigmaS(sigmaS);
        fast.filter(*out);

        checkOutput<T>(out, "FastBilateralFilter");
    }

    template <typename T>
    void doFastBilateralFilterOMP(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        if (!in->isOrganized())
        {
            throw std::invalid_argument("input cloud must be organized (height > 1) for FastBilateralFilterOMP");
        }

        auto numberOfThreads = options.check("numberOfThreads", yarp::os::Value(0)).asInt32();
        auto sigmaR = options.check("sigmaR", yarp::os::Value(0.05f)).asFloat32();
        auto sigmaS = options.check("sigmaS", yarp::os::Value(15.0f)).asFloat32();

        pcl::FastBilateralFilterOMP<T> fast;
        fast.setInputCloud(in);
        fast.setNumberOfThreads(numberOfThreads);
        fast.setSigmaR(sigmaR);
        fast.setSigmaS(sigmaS);
        fast.filter(*out);

        checkOutput<T>(out, "FastBilateralFilterOMP");
    }

    template <typename T>
    void doGreedyProjectionTriangulation(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto consistentVertexOrdering = options.check("consistentVertexOrdering", yarp::os::Value(false)).asBool();
        auto maximumAngle = options.check("maximumAngle", yarp::os::Value(2 * M_PI / 3)).asFloat64();
        auto maximumNearestNeighbors = options.check("maximumNearestNeighbors", yarp::os::Value(100)).asInt32();
        auto maximumSurfaceAngle = options.check("maximumSurfaceAngle", yarp::os::Value(M_PI / 4)).asFloat64();
        auto minimumAngle = options.check("minimumAngle", yarp::os::Value(M_PI / 18)).asFloat64();
        auto mu = options.check("mu", yarp::os::Value(0.0)).asFloat64();
        auto normalConsistency = options.check("normalConsistency", yarp::os::Value(false)).asBool();
        auto searchRadius = options.check("searchRadius", yarp::os::Value(0.0)).asFloat64();

        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(in);

        pcl::GreedyProjectionTriangulation<T> gp3;
        gp3.setConsistentVertexOrdering(consistentVertexOrdering);
        gp3.setInputCloud(in);
        gp3.setMaximumAngle(maximumAngle);
        gp3.setMaximumSurfaceAngle(maximumSurfaceAngle);
        gp3.setMaximumNearestNeighbors(maximumNearestNeighbors);
        gp3.setMinimumAngle(minimumAngle);
        gp3.setMu(mu);
        gp3.setNormalConsistency(normalConsistency);
        gp3.setSearchMethod(tree);
        gp3.setSearchRadius(searchRadius);
        gp3.reconstruct(*out);

        checkOutput(out, "GreedyProjectionTriangulation");
    }

    template <typename T>
    void doGridProjection(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto maxBinarySearchLevel = options.check("maxBinarySearchLevel", yarp::os::Value(10)).asInt32();
        auto nearestNeighborNum = options.check("nearestNeighborNum", yarp::os::Value(50)).asInt32();
        auto paddingSize = options.check("paddingSize", yarp::os::Value(3)).asInt32();
        auto resolution = options.check("resolution", yarp::os::Value(0.001)).asFloat64();

        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(in);

        pcl::GridProjection<T> gp;
        gp.setInputCloud(in);
        gp.setMaxBinarySearchLevel(maxBinarySearchLevel);
        gp.setNearestNeighborNum(nearestNeighborNum);
        gp.setPaddingSize(paddingSize);
        gp.setResolution(resolution);
        gp.setSearchMethod(tree);
        gp.reconstruct(*out);

        checkOutput(out, "GridProjection");
    }

    template <typename T>
    void doMarchingCubesHoppe(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto distanceIgnore = options.check("distanceIgnore", yarp::os::Value(-1.0f)).asFloat32();
        auto gridResolution = options.check("gridResolution", yarp::os::Value(32)).asInt32();
        auto gridResolutionX = options.check("gridResolutionX", yarp::os::Value(gridResolution)).asInt32();
        auto gridResolutionY = options.check("gridResolutionY", yarp::os::Value(gridResolution)).asInt32();
        auto gridResolutionZ = options.check("gridResolutionZ", yarp::os::Value(gridResolution)).asInt32();
        auto isoLevel = options.check("isoLevel", yarp::os::Value(0.0f)).asFloat32();
        auto percentageExtendGrid = options.check("percentageExtendGrid", yarp::os::Value(0.0f)).asFloat32();

        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(in);

        pcl::MarchingCubesHoppe<T> hoppe;
        hoppe.setDistanceIgnore(distanceIgnore);
        hoppe.setGridResolution(gridResolutionX, gridResolutionY, gridResolutionZ);
        hoppe.setInputCloud(in);
        hoppe.setIsoLevel(isoLevel);
        hoppe.setPercentageExtendGrid(percentageExtendGrid);
        hoppe.setSearchMethod(tree);
        hoppe.reconstruct(*out);

        checkOutput(out, "MarchingCubesHoppe");
    }

    template <typename T>
    void doMarchingCubesRBF(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto gridResolution = options.check("gridResolution", yarp::os::Value(32)).asInt32();
        auto gridResolutionX = options.check("gridResolutionX", yarp::os::Value(gridResolution)).asInt32();
        auto gridResolutionY = options.check("gridResolutionY", yarp::os::Value(gridResolution)).asInt32();
        auto gridResolutionZ = options.check("gridResolutionZ", yarp::os::Value(gridResolution)).asInt32();
        auto isoLevel = options.check("isoLevel", yarp::os::Value(0.0f)).asFloat32();
        auto offSurfaceDisplacement = options.check("offSurfaceDisplacement", yarp::os::Value(0.1f)).asFloat32();
        auto percentageExtendGrid = options.check("percentageExtendGrid", yarp::os::Value(0.0f)).asFloat32();

        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(in);

        pcl::MarchingCubesRBF<T> rbf;
        rbf.setGridResolution(gridResolutionX, gridResolutionY, gridResolutionZ);
        rbf.setInputCloud(in);
        rbf.setIsoLevel(isoLevel);
        rbf.setOffSurfaceDisplacement(offSurfaceDisplacement);
        rbf.setPercentageExtendGrid(percentageExtendGrid);
        rbf.setSearchMethod(tree);
        rbf.reconstruct(*out);

        checkOutput(out, "MarchingCubesRBF");
    }

    void doMeshQuadricDecimationVTK(const pcl::PolygonMesh::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto targetReductionFactor = options.check("targetReductionFactor", yarp::os::Value(0.5f)).asFloat32();

        pcl::MeshQuadricDecimationVTK quadric;
        quadric.setInputMesh(in);
        quadric.setTargetReductionFactor(targetReductionFactor);
        quadric.process(*out);

        checkOutput(out, "MeshQuadricDecimationVTK");
    }

    void doMeshSmoothingLaplacianVTK(const pcl::PolygonMesh::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto boundarySmoothing = options.check("boundarySmoothing", yarp::os::Value(true)).asBool();
        auto convergence = options.check("convergence", yarp::os::Value(0.0f)).asFloat32();
        auto edgeAngle = options.check("edgeAngle", yarp::os::Value(15.0f)).asFloat32();
        auto featureAngle = options.check("featureAngle", yarp::os::Value(45.0f)).asFloat32();
        auto featureEdgeSmoothing = options.check("featureEdgeSmoothing", yarp::os::Value(false)).asBool();
        auto numIter = options.check("numIter", yarp::os::Value(20)).asInt32();
        auto relaxationFactor = options.check("relaxationFactor", yarp::os::Value(0.01f)).asFloat32();

        pcl::MeshSmoothingLaplacianVTK laplacian;
        laplacian.setBoundarySmoothing(boundarySmoothing);
        laplacian.setConvergence(convergence);
        laplacian.setEdgeAngle(edgeAngle);
        laplacian.setFeatureAngle(featureAngle);
        laplacian.setFeatureEdgeSmoothing(featureEdgeSmoothing);
        laplacian.setInputMesh(in);
        laplacian.setNumIter(numIter);
        laplacian.setRelaxationFactor(relaxationFactor);
        laplacian.process(*out);

        checkOutput(out, "MeshSmoothingLaplacianVTK");
    }

    void doMeshSmoothingWindowedSincVTK(const pcl::PolygonMesh::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto boundarySmoothing = options.check("boundarySmoothing", yarp::os::Value(true)).asBool();
        auto edgeAngle = options.check("edgeAngle", yarp::os::Value(15.0f)).asFloat32();
        auto featureAngle = options.check("featureAngle", yarp::os::Value(45.0f)).asFloat32();
        auto featureEdgeSmoothing = options.check("featureEdgeSmoothing", yarp::os::Value(false)).asBool();
        auto normalizeCoordinates = options.check("normalizeCoordinates", yarp::os::Value(false)).asBool();
        auto numIter = options.check("numIter", yarp::os::Value(20)).asInt32();
        auto passBand = options.check("passBand", yarp::os::Value(0.1f)).asFloat32();

        pcl::MeshSmoothingWindowedSincVTK windowed;
        windowed.setBoundarySmoothing(boundarySmoothing);
        windowed.setEdgeAngle(edgeAngle);
        windowed.setFeatureAngle(featureAngle);
        windowed.setFeatureEdgeSmoothing(featureEdgeSmoothing);
        windowed.setInputMesh(in);
        windowed.setNormalizeCoordinates(normalizeCoordinates);
        windowed.setNumIter(numIter);
        windowed.setPassBand(passBand);
        windowed.process(*out);

        checkOutput(out, "MeshSmoothingWindowedSincVTK");
    }

    void doMeshSubdivisionVTK(const pcl::PolygonMesh::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto filterTypeStr = options.check("filterType", yarp::os::Value("linear")).asString();

        pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType filterType;

        if (filterTypeStr == "butterfly")
        {
            filterType = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::BUTTERFLY;
        }
        else if (filterTypeStr == "linear")
        {
            filterType = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::LINEAR;
        }
        else if (filterTypeStr == "loop")
        {
            filterType = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::LOOP;
        }
        else
        {
            throw std::invalid_argument("unknown filter type: " + filterTypeStr);
        }

        pcl::MeshSubdivisionVTK subdivision;
        subdivision.setFilterType(filterType);
        subdivision.setInputMesh(in);
        subdivision.process(*out);

        checkOutput(out, "MeshSubdivisionVTK");
    }

    template <typename T>
    void doMovingLeastSquares(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto cacheMlsResults = options.check("cacheMlsResults", yarp::os::Value(true)).asBool();
        auto dilationIterations = options.check("dilationIterations", yarp::os::Value(0)).asInt32();
        auto dilationVoxelSize = options.check("dilationVoxelSize", yarp::os::Value(1.0f)).asFloat32();
        auto numberOfThreads = options.check("numberOfThreads", yarp::os::Value(1)).asInt32();
        auto pointDensity = options.check("pointDensity", yarp::os::Value(0)).asInt32();
        auto polynomialOrder = options.check("polynomialOrder", yarp::os::Value(2)).asInt32();
        auto projectionMethodStr = options.check("projectionMethod", yarp::os::Value("simple")).asString();
        auto searchRadius = options.check("searchRadius", yarp::os::Value(0.0)).asFloat64();
        auto sqrGaussParam = options.check("sqrGaussParam", yarp::os::Value(0.0)).asFloat64();
        auto upsamplingMethodStr = options.check("upsamplingMethod", yarp::os::Value("none")).asString();
        auto upsamplingRadius = options.check("upsamplingRadius", yarp::os::Value(0.0)).asFloat64();
        auto upsamplingStepSize = options.check("upsamplingStepSize", yarp::os::Value(0.0)).asFloat64();

        pcl::MLSResult::ProjectionMethod projectionMethod;

        if (projectionMethodStr == "none")
        {
            projectionMethod = pcl::MLSResult::ProjectionMethod::NONE;
        }
        else if (projectionMethodStr == "orthogonal")
        {
            projectionMethod = pcl::MLSResult::ProjectionMethod::ORTHOGONAL;
        }
        else if (projectionMethodStr == "simple")
        {
            projectionMethod = pcl::MLSResult::ProjectionMethod::SIMPLE;
        }
        else
        {
            throw std::invalid_argument("unknown projection method: " + projectionMethodStr);
        }

        typename pcl::MovingLeastSquares<T, T>::UpsamplingMethod upsamplingMethod;

        if (upsamplingMethodStr == "distinctCloud")
        {
            upsamplingMethod = decltype(upsamplingMethod)::DISTINCT_CLOUD;
        }
        else if (upsamplingMethodStr == "none")
        {
            upsamplingMethod = decltype(upsamplingMethod)::NONE;
        }
        else if (upsamplingMethodStr == "randomUniformDensity")
        {
            upsamplingMethod = decltype(upsamplingMethod)::RANDOM_UNIFORM_DENSITY;
        }
        else if (upsamplingMethodStr == "sampleLocalPlane")
        {
            upsamplingMethod = decltype(upsamplingMethod)::SAMPLE_LOCAL_PLANE;
        }
        else if (upsamplingMethodStr == "voxelGridDilation")
        {
            upsamplingMethod = decltype(upsamplingMethod)::VOXEL_GRID_DILATION;
        }
        else
        {
            throw std::invalid_argument("unknown upsampling method: " + upsamplingMethodStr);
        }

        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(in);

        pcl::MovingLeastSquares<T, T> mls;
        mls.setCacheMLSResults(cacheMlsResults);
        mls.setComputeNormals(false); // don't store normals
        mls.setDilationIterations(dilationIterations);
        mls.setDilationVoxelSize(dilationVoxelSize);
        mls.setInputCloud(in);
        mls.setNumberOfThreads(numberOfThreads);
        mls.setPointDensity(pointDensity);
        mls.setPolynomialOrder(polynomialOrder);
        mls.setProjectionMethod(projectionMethod);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(searchRadius);
        mls.setSqrGaussParam(sqrGaussParam);
        mls.setUpsamplingMethod(upsamplingMethod);
        mls.setUpsamplingRadius(upsamplingRadius);
        mls.setUpsamplingStepSize(upsamplingStepSize);
        mls.process(*out);

        checkOutput<T>(out, "MovingLeastSquares");
    }

    template <typename T1, typename T2>
    void doNormalEstimation(const typename pcl::PointCloud<T1>::ConstPtr & in, const typename pcl::PointCloud<T2>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto kSearch = options.check("kSearch", yarp::os::Value(0)).asInt32();
        auto radiusSearch = options.check("radiusSearch", yarp::os::Value(0.0)).asFloat64();

        typename pcl::search::KdTree<T1>::Ptr tree(new pcl::search::KdTree<T1>());
        tree->setInputCloud(in);

        pcl::NormalEstimation<T1, T2> estimator;
        estimator.setInputCloud(in);
        estimator.setKSearch(kSearch);
        estimator.setRadiusSearch(radiusSearch);
        estimator.setSearchMethod(tree);
        estimator.compute(*out);

        checkOutput<T2>(out, "NormalEstimation");
    }

    template <typename T1, typename T2>
    void doNormalEstimationOMP(const typename pcl::PointCloud<T1>::ConstPtr & in, const typename pcl::PointCloud<T2>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto kSearch = options.check("kSearch", yarp::os::Value(0)).asInt32();
        auto numberOfThreads = options.check("numberOfThreads", yarp::os::Value(0)).asInt32();
        auto radiusSearch = options.check("radiusSearch", yarp::os::Value(0.0)).asFloat64();

        typename pcl::search::KdTree<T1>::Ptr tree(new pcl::search::KdTree<T1>());
        tree->setInputCloud(in);

        pcl::NormalEstimationOMP<T1, T2> estimator;
        estimator.setInputCloud(in);
        estimator.setKSearch(kSearch);
        estimator.setNumberOfThreads(numberOfThreads);
        estimator.setRadiusSearch(radiusSearch);
        estimator.setSearchMethod(tree);
        estimator.compute(*out);

        checkOutput<T2>(out, "NormalEstimationOMP");
    }

    template <typename T>
    void doOrganizedFastMesh(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        if (!in->isOrganized())
        {
            throw std::invalid_argument("input cloud must be organized (height > 1) for OrganizedFastMesh");
        }

        auto angleTolerance = options.check("angleTolerance", yarp::os::Value(12.5 * M_PI / 180)).asFloat32();
        auto depthDependent = options.check("depthDependent", yarp::os::Value(false)).asBool();
        auto distanceTolerance = options.check("distanceTolerance", yarp::os::Value(-1.0f)).asFloat32();
        auto maxEdgeLengthA = options.check("maxEdgeLengthA", yarp::os::Value(0.0f)).asFloat32();
        auto maxEdgeLengthB = options.check("maxEdgeLengthB", yarp::os::Value(0.0f)).asFloat32();
        auto maxEdgeLengthC = options.check("maxEdgeLengthC", yarp::os::Value(0.0f)).asFloat32();
        auto storeShadowedFaces = options.check("storeShadowedFaces", yarp::os::Value(false)).asBool();
        auto trianglePixelSize = options.check("trianglePixelSize", yarp::os::Value(1)).asInt32();
        auto trianglePixelSizeColumns = options.check("trianglePixelSizeColumns", yarp::os::Value(trianglePixelSize)).asInt32();
        auto trianglePixelSizeRows = options.check("trianglePixelSizeRows", yarp::os::Value(trianglePixelSize)).asInt32();
        auto triangulationTypeStr = options.check("triangulationType", yarp::os::Value("quadMesh")).asString();
        auto useDepthAsDistance = options.check("useDepthAsDistance", yarp::os::Value(false)).asBool();

        typename pcl::OrganizedFastMesh<T>::TriangulationType triangulationType;

        if (triangulationTypeStr == "quadMesh")
        {
            triangulationType = decltype(triangulationType)::QUAD_MESH;
        }
        else if (triangulationTypeStr == "triangleAdaptiveCut")
        {
            triangulationType = decltype(triangulationType)::TRIANGLE_ADAPTIVE_CUT;
        }
        else if (triangulationTypeStr == "triangleLeftCut")
        {
            triangulationType = decltype(triangulationType)::TRIANGLE_LEFT_CUT;
        }
        else if (triangulationTypeStr == "triangleRightCut")
        {
            triangulationType = decltype(triangulationType)::TRIANGLE_RIGHT_CUT;
        }
        else
        {
            throw std::invalid_argument("unknown triangulation type: " + triangulationTypeStr);
        }

        pcl::OrganizedFastMesh<T> organized;
        organized.setAngleTolerance(angleTolerance);
        organized.setDistanceTolerance(distanceTolerance, depthDependent);
        organized.setInputCloud(in);
        organized.setMaxEdgeLength(maxEdgeLengthA, maxEdgeLengthB, maxEdgeLengthC);
        organized.setTrianglePixelSize(trianglePixelSize);
        organized.setTrianglePixelSizeColumns(trianglePixelSizeColumns);
        organized.setTrianglePixelSizeRows(trianglePixelSizeRows);
        organized.setTriangulationType(triangulationType);
        organized.storeShadowedFaces(storeShadowedFaces);
        organized.useDepthAsDistance(useDepthAsDistance);
        organized.reconstruct(*out);

        checkOutput(out, "OrganizedFastMesh");
    }

    template <typename T>
    void doPoisson(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto confidence = options.check("confidence", yarp::os::Value(false)).asBool();
        auto degree = options.check("degree", yarp::os::Value(2)).asInt32();
        auto depth = options.check("depth", yarp::os::Value(8)).asInt32();
        auto isoDivide = options.check("isoDivide", yarp::os::Value(8)).asInt32();
        auto manifold = options.check("manifold", yarp::os::Value(true)).asBool();
        auto minDepth = options.check("minDepth", yarp::os::Value(5)).asInt32();
        auto outputPolygons = options.check("outputPolygons", yarp::os::Value(false)).asBool();
        auto pointWeight = options.check("pointWeight", yarp::os::Value(4.0f)).asFloat32();
        auto samplesPerNode = options.check("samplesPerNode", yarp::os::Value(1.0f)).asFloat32();
        auto scale = options.check("scale", yarp::os::Value(1.1f)).asFloat32();
        auto solverDivide = options.check("solverDivide", yarp::os::Value(8)).asInt32();
#if PCL_VERSION_COMPARE(>=, 1, 12, 0)
        auto threads = options.check("threads", yarp::os::Value(1)).asInt32();
#endif

        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(in);

        pcl::Poisson<T> poisson;
        poisson.setConfidence(confidence);
        poisson.setDegree(degree);
        poisson.setDepth(depth);
        poisson.setInputCloud(in);
        poisson.setIsoDivide(isoDivide);
        poisson.setManifold(manifold);
        poisson.setMinDepth(minDepth);
        poisson.setOutputPolygons(outputPolygons);
        poisson.setPointWeight(pointWeight);
        poisson.setSamplesPerNode(samplesPerNode);
        poisson.setScale(scale);
        poisson.setSearchMethod(tree);
        poisson.setSolverDivide(solverDivide);
#if PCL_VERSION_COMPARE(>=, 1, 12, 0)
        poisson.setThreads(threads);
#endif
        poisson.reconstruct(*out);

        checkOutput(out, "Poisson");
    }

    void doSimplificationRemoveUnusedVertices(const pcl::PolygonMesh::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        pcl::surface::SimplificationRemoveUnusedVertices cleaner;
        cleaner.simplify(*in, *out);
        checkOutput(out, "doSimplificationRemoveUnusedVertices");
    }

    template <typename T>
    void doUniformSampling(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto radiusSearch = options.check("radiusSearch", yarp::os::Value(0.0)).asFloat64();

        pcl::UniformSampling<T> uniform;
        uniform.setInputCloud(in);
        uniform.setRadiusSearch(radiusSearch);
        uniform.filter(*out);

        checkOutput<T>(out, "UniformSampling");
    }

    template <typename T>
    void doVoxelGrid(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto downsampleAllData = options.check("downsampleAllData", yarp::os::Value(true)).asBool();
        auto leafSize = options.check("leafSize", yarp::os::Value(0.0f)).asFloat32();
        auto leafSizeX = options.check("leafSizeX", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeY = options.check("leafSizeY", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeZ = options.check("leafSizeZ", yarp::os::Value(leafSize)).asFloat32();
        auto limitMax = options.check("limitMax", yarp::os::Value(FLT_MAX)).asFloat64();
        auto limitMin = options.check("limitMin", yarp::os::Value(-FLT_MAX)).asFloat64();
        auto limitsNegative = options.check("limitsNegative", yarp::os::Value(false)).asBool();
        auto minimumPointsNumberPerVoxel = options.check("minimumPointsNumberPerVoxel", yarp::os::Value(0)).asInt32();

        pcl::VoxelGrid<T> grid;
        grid.setDownsampleAllData(downsampleAllData);
        grid.setFilterLimits(limitMin, limitMax);
        grid.setFilterLimitsNegative(limitsNegative);
        grid.setInputCloud(in);
        grid.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
        grid.setMinimumPointsNumberPerVoxel(minimumPointsNumberPerVoxel);
        grid.setSaveLeafLayout(false);
        grid.filter(*out);

        checkOutput<T>(out, "VoxelGrid");
    }

    template <typename T>
    void processStep(const cloud_container & prev, cloud_container & curr, const yarp::os::Searchable & options)
    {
        using any_xyz_t = typename pcl_convert<T, pcl_all_xyz_types_tag>::type;
        using xyz_rgba_t = typename pcl_convert<T, pcl_xyz_rgba_types_tag>::type;
        using normal_t = typename pcl_convert<T, pcl_normal_types_tag>::type;

        if (!options.check("algorithm"))
        {
            throw std::invalid_argument("missing algorithm parameter");
        }

        auto algorithm = options.find("algorithm").asString();

        switch (makeHash(algorithm))
        {
        case "ApproximateVoxelGrid"_hash:
            doApproximateVoxelGrid<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
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
            doFastBilateralFilter<xyz_rgba_t>(prev.getCloud<xyz_rgba_t>(), curr.setCloud<xyz_rgba_t>(), options);
            break;
        case "FastBilateralFilterOMP"_hash:
            doFastBilateralFilterOMP<xyz_rgba_t>(prev.getCloud<xyz_rgba_t>(), curr.setCloud<xyz_rgba_t>(), options);
            break;
        case "GreedyProjectionTriangulation"_hash:
            doGreedyProjectionTriangulation<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "GridProjection"_hash:
            doGridProjection<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "MarchingCubesHoppe"_hash:
            doMarchingCubesHoppe<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "MarchingCubesRBF"_hash:
            doMarchingCubesRBF<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
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
        case "MovingLeastSquares"_hash:
            doMovingLeastSquares<any_xyz_t>(prev.getCloud<any_xyz_t>(), curr.setCloud<any_xyz_t>(), options);
            break;
        case "NormalEstimation"_hash:
            pcl::copyPointCloud(*prev.getCloud<any_xyz_t>(), *curr.setCloud<normal_t>());
            doNormalEstimation<any_xyz_t, normal_t>(prev.getCloud<any_xyz_t>(), curr.useCloud<normal_t>(), options);
            break;
        case "NormalEstimationOMP"_hash:
            pcl::copyPointCloud(*prev.getCloud<any_xyz_t>(), *curr.setCloud<normal_t>());
            doNormalEstimationOMP<any_xyz_t, normal_t>(prev.getCloud<any_xyz_t>(), curr.useCloud<normal_t>(), options);
            break;
        case "OrganizedFastMesh"_hash:
            doOrganizedFastMesh<xyz_rgba_t>(prev.getCloud<xyz_rgba_t>(), curr.setMesh(), options);
            break;
        case "Poisson"_hash:
            doPoisson<normal_t>(prev.getCloud<normal_t>(), curr.setMesh(), options);
            break;
        case "SimplificationRemoveUnusedVertices"_hash:
            doSimplificationRemoveUnusedVertices(prev.getMesh(), curr.setMesh(), options);
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
    void meshFromCloudPCL(const typename pcl::PointCloud<T>::Ptr &, pcl::PolygonMesh::Ptr &, const yarp::sig::VectorOf<yarp::os::Property> &)
    {}

    template <typename T, std::enable_if_t<!is_unsupported_type<T>, bool> = true>
    void meshFromCloudPCL(const typename pcl::PointCloud<T>::Ptr & cloud, pcl::PolygonMesh::Ptr & mesh, const yarp::sig::VectorOf<yarp::os::Property> & options)
    {
        cloud_container data;
        data.setCloud<T>() = cloud;

        for (const auto & step : options)
        {
            cloud_container temp;
            processStep<T>(data, temp, step);
            data = std::move(temp);
        }

        mesh = data.getMesh();
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
                    options.emplace_back(groupConfig.toString().c_str());
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

    if (is_unsupported_type<pcl_input_type>)
    {
        yError() << "unsupported input point type";
        return false;
    }

    if (is_unsupported_type<pcl_output_type>)
    {
        yError() << "unsupported output point type";
        return false;
    }

    if (options.size() == 0)
    {
        yError() << "empty configuration";
        return false;
    }

    // Convert YARP cloud to PCL cloud.
    typename pcl::PointCloud<pcl_input_type>::Ptr pclXYZ(new pcl::PointCloud<pcl_input_type>());
    yarp::pcl::toPCL(cloud, *pclXYZ);

    // Perform surface reconstruction.
    pcl::PolygonMesh::Ptr pclMesh;

    try
    {
        meshFromCloudPCL<pcl_input_type>(pclXYZ, pclMesh, options);
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

} // namespace YarpCloudUtils

} // namespace roboticslab

#ifdef HAVE_PCL
// explicit instantiations
#include "YarpCloudUtils-pcl-inst.hpp"
#endif
