// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCloudUtils.hpp"

#include <yarp/os/LogStream.h>
#ifdef HAVE_PCL
#include <yarp/pcl/Pcl.h>

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>

#include <exception>
#include <stdexcept>
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
        typename pcl::PointCloud<T>::ConstPtr getAsInput() const;

        template <typename T>
        typename pcl::PointCloud<T>::Ptr & getAsOutput();

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb;
        pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi;
        pcl::PointCloud<pcl::InterestPoint>::Ptr xyz_interest;
        pcl::PointCloud<pcl::PointNormal>::Ptr xyz_normal;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr xyz_rgb_normal;
    };

    template <typename T>
    typename pcl::PointCloud<T>::ConstPtr cloud_container::getAsInput() const
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
        else
            throw std::runtime_error("cloud pointer was not initialized");
    }

    template <>
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_container::getAsOutput<pcl::PointXYZ>()
    { return xyz.reset(new pcl::PointCloud<pcl::PointXYZ>()), xyz; }

    template <>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_container::getAsOutput<pcl::PointXYZRGB>()
    { return xyz_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>()), xyz_rgb; }

    template <>
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_container::getAsOutput<pcl::PointXYZI>()
    { return xyzi.reset(new pcl::PointCloud<pcl::PointXYZI>()), xyzi; }

    template <>
    pcl::PointCloud<pcl::InterestPoint>::Ptr & cloud_container::getAsOutput<pcl::InterestPoint>()
    { return xyz_interest.reset(new pcl::PointCloud<pcl::InterestPoint>()), xyz_interest; }

    template <>
    pcl::PointCloud<pcl::PointNormal>::Ptr & cloud_container::getAsOutput<pcl::PointNormal>()
    { return xyz_normal.reset(new pcl::PointCloud<pcl::PointNormal>()), xyz_normal; }

    template <>
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud_container::getAsOutput<pcl::PointXYZRGBNormal>()
    { return xyz_rgb_normal.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>()), xyz_rgb_normal; }

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
        auto downsampleAllData = options.check("downsampleDownsampleAllData", yarp::os::Value(true)).asBool();
        auto leafSize = options.check("downsampleLeafSize", yarp::os::Value(0.0f)).asFloat32();
        auto leafSizeX = options.check("downsampleLeafSizeX", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeY = options.check("downsampleLeafSizeY", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeZ = options.check("downsampleLeafSizeZ", yarp::os::Value(leafSize)).asFloat32();

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
        auto alpha = options.check("surfaceAlpha", yarp::os::Value(0.0)).asFloat64();

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
        auto maxX = options.check("cropMaxX", yarp::os::Value(0.0f)).asFloat32();
        auto maxY = options.check("cropMaxY", yarp::os::Value(0.0f)).asFloat32();
        auto maxZ = options.check("cropMaxZ", yarp::os::Value(0.0f)).asFloat32();
        auto minX = options.check("cropMinX", yarp::os::Value(0.0f)).asFloat32();
        auto minY = options.check("cropMinY", yarp::os::Value(0.0f)).asFloat32();
        auto minZ = options.check("cropMinZ", yarp::os::Value(0.0f)).asFloat32();
        auto negative = options.check("cropNegative", yarp::os::Value(false)).asBool();
        auto rotationX = options.check("cropRotationX", yarp::os::Value(0.0f)).asFloat32();
        auto rotationY = options.check("cropRotationY", yarp::os::Value(0.0f)).asFloat32();
        auto rotationZ = options.check("cropRotationZ", yarp::os::Value(0.0f)).asFloat32();
        auto translationX = options.check("cropTranslationX", yarp::os::Value(0.0f)).asFloat32();
        auto translationY = options.check("cropTranslationY", yarp::os::Value(0.0f)).asFloat32();
        auto translationZ = options.check("cropTranslationZ", yarp::os::Value(0.0f)).asFloat32();

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
        auto sigmaR = options.check("smoothSigmaR", yarp::os::Value(0.05f)).asFloat32();
        auto sigmaS = options.check("smoothSigmaS", yarp::os::Value(15.0f)).asFloat32();

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
        auto numberOfThreads = options.check("smoothNumberOfThreads", yarp::os::Value(0)).asInt32();
        auto sigmaR = options.check("smoothSigmaR", yarp::os::Value(0.05f)).asFloat32();
        auto sigmaS = options.check("smoothSigmaS", yarp::os::Value(15.0f)).asFloat32();

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
        auto consistentVertexOrdering = options.check("surfaceConsistentVertexOrdering", yarp::os::Value(false)).asBool();
        auto maximumAngle = options.check("surfaceMaximumAngle", yarp::os::Value(2 * M_PI / 3)).asFloat64();
        auto maximumNearestNeighbors = options.check("surfaceMaximumNearestNeighbors", yarp::os::Value(100)).asInt32();
        auto maximumSurfaceAngle = options.check("surfaceMaximumSurfaceAngle", yarp::os::Value(M_PI / 4)).asFloat64();
        auto minimumAngle = options.check("surfaceMinimumAngle", yarp::os::Value(M_PI / 18)).asFloat64();
        auto mu = options.check("surfaceMu", yarp::os::Value(0.0)).asFloat64();
        auto normalConsistency = options.check("surfaceNormalConsistency", yarp::os::Value(false)).asBool();
        auto searchRadius = options.check("surfaceSearchRadius", yarp::os::Value(0.0)).asFloat64();

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
        auto maxBinarySearchLevel = options.check("surfaceMaxBinarySearchLevel", yarp::os::Value(10)).asInt32();
        auto nearestNeighborNum = options.check("surfaceNearestNeighborNum", yarp::os::Value(50)).asInt32();
        auto paddingSize = options.check("surfacePaddingSize", yarp::os::Value(3)).asInt32();
        auto resolution = options.check("surfaceResolution", yarp::os::Value(0.001)).asFloat64();

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
        auto distanceIgnore = options.check("surfaceDistanceIgnore", yarp::os::Value(-1.0f)).asFloat32();
        auto gridResolution = options.check("surfaceGridResolution", yarp::os::Value(32)).asInt32();
        auto gridResolutionX = options.check("surfaceGridResolutionX", yarp::os::Value(gridResolution)).asInt32();
        auto gridResolutionY = options.check("surfaceGridResolutionY", yarp::os::Value(gridResolution)).asInt32();
        auto gridResolutionZ = options.check("surfaceGridResolutionZ", yarp::os::Value(gridResolution)).asInt32();
        auto isoLevel = options.check("surfaceIsoLevel", yarp::os::Value(0.0f)).asFloat32();
        auto percentageExtendGrid = options.check("surfacePercentageExtendGrid", yarp::os::Value(0.0f)).asFloat32();

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
        auto gridResolution = options.check("surfaceGridResolution", yarp::os::Value(32)).asInt32();
        auto gridResolutionX = options.check("surfaceGridResolutionX", yarp::os::Value(gridResolution)).asInt32();
        auto gridResolutionY = options.check("surfaceGridResolutionY", yarp::os::Value(gridResolution)).asInt32();
        auto gridResolutionZ = options.check("surfaceGridResolutionZ", yarp::os::Value(gridResolution)).asInt32();
        auto isoLevel = options.check("surfaceIsoLevel", yarp::os::Value(0.0f)).asFloat32();
        auto offSurfaceDisplacement = options.check("surfaceOffSurfaceDisplacement", yarp::os::Value(0.1f)).asFloat32();
        auto percentageExtendGrid = options.check("surfacePercentageExtendGrid", yarp::os::Value(0.0f)).asFloat32();

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
        auto targetReductionFactor = options.check("processTargetReductionFactor", yarp::os::Value(0.5f)).asFloat32();

        pcl::MeshQuadricDecimationVTK quadric;
        quadric.setInputMesh(in);
        quadric.setTargetReductionFactor(targetReductionFactor);
        quadric.process(*out);

        checkOutput(out, "MeshQuadricDecimationVTK");
    }

    void doMeshSmoothingLaplacianVTK(const pcl::PolygonMesh::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto boundarySmoothing = options.check("processBoundarySmoothing", yarp::os::Value(true)).asBool();
        auto convergence = options.check("processConvergence", yarp::os::Value(0.0f)).asFloat32();
        auto edgeAngle = options.check("processEdgeAngle", yarp::os::Value(15.0f)).asFloat32();
        auto featureAngle = options.check("processFeatureAngle", yarp::os::Value(45.0f)).asFloat32();
        auto featureEdgeSmoothing = options.check("processFeatureEdgeSmoothing", yarp::os::Value(false)).asBool();
        auto numIter = options.check("processNumIter", yarp::os::Value(20)).asInt32();
        auto relaxationFactor = options.check("processRelaxationFactor", yarp::os::Value(0.01f)).asFloat32();

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
        auto boundarySmoothing = options.check("processBoundarySmoothing", yarp::os::Value(true)).asBool();
        auto edgeAngle = options.check("processEdgeAngle", yarp::os::Value(15.0f)).asFloat32();
        auto featureAngle = options.check("processFeatureAngle", yarp::os::Value(45.0f)).asFloat32();
        auto featureEdgeSmoothing = options.check("processFeatureEdgeSmoothing", yarp::os::Value(false)).asBool();
        auto normalizeCoordinates = options.check("processNormalizeCoordinates", yarp::os::Value(false)).asBool();
        auto numIter = options.check("processNumIter", yarp::os::Value(20)).asInt32();
        auto passBand = options.check("processPassBand", yarp::os::Value(0.1f)).asFloat32();

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
        auto filterTypeStr = options.check("processFilterType", yarp::os::Value("linear")).asString();

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
            throw std::invalid_argument("unknown filter type for mesh processing step: " + filterTypeStr);
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
        auto cacheMlsResults = options.check("smoothCacheMlsResults", yarp::os::Value(true)).asBool();
        auto dilationIterations = options.check("smoothDilationIterations", yarp::os::Value(0)).asInt32();
        auto dilationVoxelSize = options.check("smoothDilationVoxelSize", yarp::os::Value(1.0f)).asFloat32();
        auto numberOfThreads = options.check("smoothNumberOfThreads", yarp::os::Value(1)).asInt32();
        auto pointDensity = options.check("smoothPointDensity", yarp::os::Value(0)).asInt32();
        auto polynomialOrder = options.check("smoothPolynomialOrder", yarp::os::Value(2)).asInt32();
        auto projectionMethodStr = options.check("smoothProjectionMethod", yarp::os::Value("simple")).asString();
        auto searchRadius = options.check("smoothSearchRadius", yarp::os::Value(0.0)).asFloat64();
        auto sqrGaussParam = options.check("smoothSqrGaussParam", yarp::os::Value(0.0)).asFloat64();
        auto upsamplingMethodStr = options.check("smoothUpsamplingMethod", yarp::os::Value("none")).asString();
        auto upsamplingRadius = options.check("smoothUpsamplingRadius", yarp::os::Value(0.0)).asFloat64();
        auto upsamplingStepSize = options.check("smoothUpsamplingStepSize", yarp::os::Value(0.0)).asFloat64();

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
            throw std::invalid_argument("unknown projection method for cloud smoothing step: " + projectionMethodStr);
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
            throw std::invalid_argument("unknown upsampling method for cloud smoothing step: " + upsamplingMethodStr);
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

    template <typename T>
    void doNormalEstimation(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PointCloud<pcl::Normal>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto kSearch = options.check("estimatorKSearch", yarp::os::Value(0)).asInt32();
        auto radiusSearch = options.check("estimatorRadiusSearch", yarp::os::Value(0.0)).asFloat64();

        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(in);

        pcl::NormalEstimation<T, pcl::Normal> estimator;
        estimator.setInputCloud(in);
        estimator.setKSearch(kSearch);
        estimator.setRadiusSearch(radiusSearch);
        estimator.setSearchMethod(tree);
        estimator.compute(*out);

        checkOutput<pcl::Normal>(out, "NormalEstimation");
    }

    template <typename T>
    void doNormalEstimationOMP(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PointCloud<pcl::Normal>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto kSearch = options.check("estimatorKSearch", yarp::os::Value(0)).asInt32();
        auto numberOfThreads = options.check("estimatorNumberOfThreads", yarp::os::Value(0)).asInt32();
        auto radiusSearch = options.check("estimatorRadiusSearch", yarp::os::Value(0.0)).asFloat64();

        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(in);

        pcl::NormalEstimationOMP<T, pcl::Normal> estimator;
        estimator.setInputCloud(in);
        estimator.setKSearch(kSearch);
        estimator.setNumberOfThreads(numberOfThreads);
        estimator.setRadiusSearch(radiusSearch);
        estimator.setSearchMethod(tree);
        estimator.compute(*out);

        checkOutput<pcl::Normal>(out, "NormalEstimationOMP");
    }

    template <typename T>
    void doOrganizedFastMesh(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        if (!in->isOrganized())
        {
            throw std::invalid_argument("input cloud must be organized (height > 1)");
        }

        pcl::OrganizedFastMesh<T> organized;

        auto angleTolerance = options.check("surfaceAngleTolerance", yarp::os::Value(12.5 * M_PI / 180)).asFloat32();
        auto depthDependent = options.check("surfaceDepthDependent", yarp::os::Value(false)).asBool();
        auto distanceTolerance = options.check("surfaceDistanceTolerance", yarp::os::Value(-1.0f)).asFloat32();
        auto maxEdgeLengthA = options.check("surfaceMaxEdgeLengthA", yarp::os::Value(0.0f)).asFloat32();
        auto maxEdgeLengthB = options.check("surfaceMaxEdgeLengthB", yarp::os::Value(0.0f)).asFloat32();
        auto maxEdgeLengthC = options.check("surfaceMaxEdgeLengthC", yarp::os::Value(0.0f)).asFloat32();
        auto storeShadowedFaces = options.check("surfaceStoreShadowedFaces", yarp::os::Value(false)).asBool();
        auto trianglePixelSize = options.check("surfaceTrianglePixelSize", yarp::os::Value(1)).asInt32();
        auto trianglePixelSizeColumns = options.check("surfaceTrianglePixelSizeColumns", yarp::os::Value(trianglePixelSize)).asInt32();
        auto trianglePixelSizeRows = options.check("surfaceTrianglePixelSizeRows", yarp::os::Value(trianglePixelSize)).asInt32();
        auto triangulationTypeStr = options.check("surfaceTriangulationType", yarp::os::Value("quadMesh")).asString();
        auto useDepthAsDistance = options.check("surfaceUseDepthAsDistance", yarp::os::Value(false)).asBool();

        typename decltype(organized)::TriangulationType triangulationType;

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
            throw std::invalid_argument("unknown triangulation type for reconstruct step: " + triangulationTypeStr);
        }

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
        auto confidence = options.check("surfaceConfidence", yarp::os::Value(false)).asBool();
        auto degree = options.check("surfaceDegree", yarp::os::Value(2)).asInt32();
        auto depth = options.check("surfaceDepth", yarp::os::Value(8)).asInt32();
        auto isoDivide = options.check("surfaceIsoDivide", yarp::os::Value(8)).asInt32();
        auto manifold = options.check("surfaceManifold", yarp::os::Value(true)).asBool();
        auto minDepth = options.check("surfaceMinDepth", yarp::os::Value(5)).asInt32();
        auto outputPolygons = options.check("surfaceOutputPolygons", yarp::os::Value(false)).asBool();
        auto pointWeight = options.check("surfacePointWeight", yarp::os::Value(4.0f)).asFloat32();
        auto samplesPerNode = options.check("surfaceSamplesPerNode", yarp::os::Value(1.0f)).asFloat32();
        auto scale = options.check("surfaceScale", yarp::os::Value(1.1f)).asFloat32();
        auto solverDivide = options.check("surfaceSolverDivide", yarp::os::Value(8)).asInt32();
#if PCL_VERSION_COMPARE(>=, 1, 12, 0)
        auto threads = options.check("surfaceThreads", yarp::os::Value(1)).asInt32();
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

    template <typename T>
    void doUniformSampling(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto radiusSearch = options.check("downsampleRadiusSearch", yarp::os::Value(0.0)).asFloat64();

        pcl::UniformSampling<T> uniform;
        uniform.setInputCloud(in);
        uniform.setRadiusSearch(radiusSearch);
        uniform.filter(*out);

        checkOutput<T>(out, "UniformSampling");
    }

    template <typename T>
    void doVoxelGrid(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto downsampleAllData = options.check("downsampleDownsampleAllData", yarp::os::Value(true)).asBool();
        auto leafSize = options.check("downsampleLeafSize", yarp::os::Value(0.0f)).asFloat32();
        auto leafSizeX = options.check("downsampleLeafSizeX", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeY = options.check("downsampleLeafSizeY", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeZ = options.check("downsampleLeafSizeZ", yarp::os::Value(leafSize)).asFloat32();
        auto limitMax = options.check("downsampleLimitMax", yarp::os::Value(FLT_MAX)).asFloat64();
        auto limitMin = options.check("downsampleLimitMin", yarp::os::Value(-FLT_MAX)).asFloat64();
        auto limitsNegative = options.check("downsampleLimitsNegative", yarp::os::Value(false)).asBool();
        auto minimumPointsNumberPerVoxel = options.check("downsampleMinimumPointsNumberPerVoxel", yarp::os::Value(0)).asInt32();

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

    template <typename T1, typename T2>
    typename pcl::PointCloud<T2>::ConstPtr tryNormalize(const typename pcl::PointCloud<T1>::ConstPtr & in, const yarp::os::Searchable & options)
    {
        if (options.check("estimatorAlgorithm"))
        {
            auto algorithm = options.find("estimatorAlgorithm").asString();

            // Estimate normals.
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

            if (algorithm == "NormalEstimation")
            {
                doNormalEstimation<T1>(in, normals, options);
            }
            else if (algorithm == "NormalEstimationOMP")
            {
                doNormalEstimationOMP<T1>(in, normals, options);
            }
            else
            {
                throw std::invalid_argument("unsupported algorithm for normal estimation step: " + algorithm);
            }

            // Concatenate point clouds.
            typename pcl::PointCloud<T2>::Ptr cloudWithNormals(new pcl::PointCloud<T2>());
            pcl::concatenateFields(*in, *normals, *cloudWithNormals);

            return cloudWithNormals;
        }
        else if (is_normal_type<T1>)
        {
            return initializeCloudPointer<T1, T2>(in);
        }
        else
        {
            throw std::invalid_argument("missing mandatory normal estimation algorithm for non-normal point type");
        }
    }

    template <typename T, std::enable_if_t<is_unsupported_type<T>, bool> = true>
    void meshFromCloudPCL(const typename pcl::PointCloud<T>::Ptr &, pcl::PolygonMesh::Ptr &, const yarp::os::Searchable &)
    {}

    template <typename T, std::enable_if_t<!is_unsupported_type<T>, bool> = true>
    void meshFromCloudPCL(const typename pcl::PointCloud<T>::Ptr & cloud, pcl::PolygonMesh::Ptr & mesh, const yarp::os::Searchable & options)
    {
        using any_xyz_t = typename pcl_convert<T, pcl_all_xyz_types_tag>::type;
        using xyz_rgba_t = typename pcl_convert<T, pcl_xyz_rgba_types_tag>::type;

        // Crop input cloud.
        cloud_container cropped;

        if (options.check("cropAlgorithm"))
        {
            auto algorithm = options.find("cropAlgorithm").asString();

            if (algorithm == "CropBox")
            {
                doCropBox<any_xyz_t>(cloud, cropped.getAsOutput<any_xyz_t>(), options);
            }
            else
            {
                throw std::invalid_argument("unsupported algorithm for crop step: " + algorithm);
            }
        }
        else
        {
            cropped.getAsOutput<T>() = cloud;
        }

        // Downsample so that further computations are actually feasible.
        cloud_container filtered;

        if (options.check("downsampleAlgorithm"))
        {
            auto algorithm = options.find("downsampleAlgorithm").asString();

            if (algorithm == "ApproximateVoxelGrid")
            {
                doApproximateVoxelGrid<any_xyz_t>(cropped.getAsInput<any_xyz_t>(), filtered.getAsOutput<any_xyz_t>(), options);
            }
            else if (algorithm == "UniformSampling")
            {
                doUniformSampling<any_xyz_t>(cropped.getAsInput<any_xyz_t>(), filtered.getAsOutput<any_xyz_t>(), options);
            }
            else if (algorithm == "VoxelGrid")
            {
                doVoxelGrid<any_xyz_t>(cropped.getAsInput<any_xyz_t>(), filtered.getAsOutput<any_xyz_t>(), options);
            }
            else
            {
                throw std::invalid_argument("unsupported algorithm for downsample step: " + algorithm);
            }
        }
        else
        {
            filtered = cropped;
        }

        // Pre-process input cloud.
        cloud_container smoothed;

        if (options.check("smoothAlgorithm"))
        {
            auto algorithm = options.find("smoothAlgorithm").asString();

            if (algorithm == "FastBilateralFilter")
            {
                doFastBilateralFilter<xyz_rgba_t>(filtered.getAsInput<xyz_rgba_t>(), smoothed.getAsOutput<xyz_rgba_t>(), options);
            }
            else if (algorithm == "FastBilateralFilterOMP")
            {
                doFastBilateralFilterOMP<xyz_rgba_t>(filtered.getAsInput<xyz_rgba_t>(), smoothed.getAsOutput<xyz_rgba_t>(), options);
            }
            else if (algorithm == "MovingLeastSquares")
            {
                doMovingLeastSquares<any_xyz_t>(filtered.getAsInput<any_xyz_t>(), smoothed.getAsOutput<any_xyz_t>(), options);
            }
            else
            {
                throw std::invalid_argument("unsupported algorithm for cloud smoothing step: " + algorithm);
            }
        }
        else
        {
            smoothed = filtered;
        }

        // Reconstruct triangle mesh.
        pcl::PolygonMesh::Ptr reconstructed(new pcl::PolygonMesh());

        if (options.check("surfaceAlgorithm"))
        {
            using normal_t = typename pcl_surface_normal<any_xyz_t>::type;
            using concatenated_t = typename pcl_concatenate_normals<normal_t>::type;

            auto algorithm = options.find("surfaceAlgorithm").asString();

            if (algorithm == "ConcaveHull")
            {
                doConcaveHull<any_xyz_t>(smoothed.getAsInput<any_xyz_t>(), reconstructed, options);
            }
            else if (algorithm == "ConvexHull")
            {
                doConvexHull<any_xyz_t>(smoothed.getAsInput<any_xyz_t>(), reconstructed, options);
            }
            else if (algorithm == "GreedyProjectionTriangulation")
            {
                auto in = smoothed.getAsInput<normal_t>();
                auto normalized = tryNormalize<normal_t, concatenated_t>(in, options);
                doGreedyProjectionTriangulation<concatenated_t>(normalized, reconstructed, options);
            }
            else if (algorithm == "GridProjection")
            {
                auto in = smoothed.getAsInput<normal_t>();
                auto normalized = tryNormalize<normal_t, concatenated_t>(in, options);
                doGridProjection<concatenated_t>(normalized, reconstructed, options);
            }
            else if (algorithm == "MarchingCubesHoppe")
            {
                auto in = smoothed.getAsInput<normal_t>();
                auto normalized = tryNormalize<normal_t, concatenated_t>(in, options);
                doMarchingCubesHoppe<concatenated_t>(normalized, reconstructed, options);
            }
            else if (algorithm == "MarchingCubesRBF")
            {
                auto in = smoothed.getAsInput<normal_t>();
                auto normalized = tryNormalize<normal_t, concatenated_t>(in, options);
                doMarchingCubesRBF<concatenated_t>(normalized, reconstructed, options);
            }
            else if (algorithm == "OrganizedFastMesh")
            {
                doOrganizedFastMesh<xyz_rgba_t>(smoothed.getAsInput<xyz_rgba_t>(), reconstructed, options);
            }
            else if (algorithm == "Poisson")
            {
                auto in = smoothed.getAsInput<normal_t>();
                auto normalized = tryNormalize<normal_t, concatenated_t>(in, options);
                doPoisson<concatenated_t>(normalized, reconstructed, options);
            }
            else
            {
                throw std::invalid_argument("unsupported algorithm for surface reconstruction step: " + algorithm);
            }
        }
        else
        {
            throw std::invalid_argument("missing mandatory surface algorithm");
        }

        // Post-process output mesh.
        pcl::PolygonMesh::Ptr processed = reconstructed;

        if (options.check("processAlgorithm"))
        {
            auto algorithm = options.find("processAlgorithm").asString();

            if (algorithm == "MeshQuadricDecimationVTK")
            {
                doMeshQuadricDecimationVTK(reconstructed, processed, options);
            }
            else if (algorithm == "MeshSmoothingLaplacianVTK")
            {
                doMeshSmoothingLaplacianVTK(reconstructed, processed, options);
            }
            else if (algorithm == "MeshSmoothingWindowedSincVTK")
            {
                doMeshSmoothingWindowedSincVTK(reconstructed, processed, options);
            }
            else if (algorithm == "MeshSubdivisionVTK")
            {
                doMeshSubdivisionVTK(reconstructed, processed, options);
            }
            else
            {
                throw std::invalid_argument("unsupported algorithm for mesh processing step: " + algorithm);
            }
        }

        // Remove unused vertices from vertex cloud.
        pcl::PolygonMesh::Ptr simplified = processed;

        if (options.check("simplifyMesh", yarp::os::Value(false)).asBool())
        {
            simplified.reset(new pcl::PolygonMesh());
            pcl::surface::SimplificationRemoveUnusedVertices cleaner;
            cleaner.simplify(*processed, *simplified);
        }

        mesh = simplified;
    }
}
#endif

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T1, typename T2>
bool meshFromCloud(const yarp::sig::PointCloud<T1> & cloud, yarp::sig::PointCloud<T2> & meshPoints, yarp::sig::VectorOf<int> & meshIndices, const yarp::os::Searchable & options)
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

    // Convert YARP cloud to PCL cloud.
    typename pcl::PointCloud<pcl_input_type>::Ptr pclXYZ(new pcl::PointCloud<pcl_input_type>());
    yarp::pcl::toPCL(cloud, *pclXYZ);

    // Perform surface reconstruction.
    pcl::PolygonMesh::Ptr pclMesh(new pcl::PolygonMesh());

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

} // namespace YarpCloudUtils

} // namespace roboticslab

#ifdef HAVE_PCL
// explicit instantiations
#include "YarpCloudUtils-pcl-inst.hpp"
#endif
