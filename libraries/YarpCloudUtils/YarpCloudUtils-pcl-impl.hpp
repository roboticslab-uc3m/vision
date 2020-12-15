// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CLOUD_UTILS_PCL_IMPL_HPP__
#define __YARP_CLOUD_UTILS_PCL_IMPL_HPP__

#define _USE_MATH_DEFINES
#include <cmath> // M_PI
#include <ctime> // std::time

#include <limits> // std::numeric_limits
#include <stdexcept> // std::invalid_argument, std::runtime_error
#include <string>

#include <pcl/pcl_config.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#if PCL_VERSION_COMPARE(>=, 1, 9, 0)
#include <pcl/surface/mls.h>
#endif
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>

namespace
{

auto getTransformation(const yarp::os::Searchable & options)
{
    auto transformation = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    const auto & translation = options.find("translation");

    if (!translation.isNull())
    {
        if (!translation.isList() || translation.asList()->size() != 3)
        {
            throw std::runtime_error("translation is not a list or size not equal to 3");
        }

        const auto * b = translation.asList();
        Eigen::Vector3d vector(b->get(0).asFloat64(), b->get(1).asFloat64(), b->get(2).asFloat64());
        transformation.translate(vector);
    }

    const auto & rotation = options.find("rotation");

    if (!rotation.isNull())
    {
        if (!rotation.isList() || rotation.asList()->size() != 3)
        {
            throw std::runtime_error("rotation is not a list or size not equal to 3");
        }

        const auto * b = rotation.asList();
        Eigen::Vector3d axis(b->get(0).asFloat64(), b->get(1).asFloat64(), b->get(2).asFloat64());
        Eigen::AngleAxisd rot(axis.norm(), axis.normalized());
        transformation.rotate(rot);
    }

    return transformation;
}

template <typename T>
void checkOutput(const typename pcl::PointCloud<T>::ConstPtr & cloud, const std::string & caller)
{
    if (cloud->empty())
    {
        throw std::runtime_error(caller + " returned an empty cloud");
    }
}

inline void checkOutput(const pcl::PolygonMesh::ConstPtr & mesh, const std::string & caller)
{
    if (mesh->cloud.data.empty() || mesh->polygons.empty())
    {
        throw std::runtime_error(caller + " returned an empty or incomplete mesh");
    }
}

template <typename T>
void doTransformPointCloud(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto transformation = getTransformation(options);
    pcl::transformPointCloud(*in, *out, transformation);
    checkOutput<T>(out, "transformPointCloud");
}

template <typename T>
void doTransformPointCloudWithNormals(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto transformation = getTransformation(options);
    pcl::transformPointCloudWithNormals(*in, *out, transformation);
    checkOutput<T>(out, "transformPointCloudWithNormals");
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
void doBilateralFilter(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto halfSize = options.check("halfSize", yarp::os::Value(0.0)).asFloat64();
    auto stdDev = options.check("stdDev", yarp::os::Value(std::numeric_limits<double>::max())).asFloat64();

    typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
    tree->setInputCloud(in);

    pcl::BilateralFilter<T> filter;
    filter.setHalfSize(halfSize);
    filter.setInputCloud(in);
    filter.setSearchMethod(tree);
    filter.setStdDev(stdDev);
    filter.filter(*out);

    checkOutput<T>(out, "BilateralFilter");
}

template <typename T>
void doBilateralUpsampling(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto sigmaColor = options.check("sigmaColor", yarp::os::Value(15.0f)).asFloat32();
    auto sigmaDepth = options.check("sigmaDepth", yarp::os::Value(0.5f)).asFloat32();
    auto windowSize = options.check("windowSize", yarp::os::Value(5)).asInt32();

    pcl::BilateralUpsampling<T, T> upsampler;
    upsampler.setInputCloud(in);
    upsampler.setSigmaColor(sigmaColor);
    upsampler.setSigmaDepth(sigmaDepth);
    upsampler.setWindowSize(windowSize);
    upsampler.process(*out);

    checkOutput<T>(out, "BilateralUpsampling");
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
    auto keepOrganized = options.check("keepOrganized", yarp::os::Value(false)).asBool();
    auto maxX = options.check("maxX", yarp::os::Value(1.0f)).asFloat32();
    auto maxY = options.check("maxY", yarp::os::Value(1.0f)).asFloat32();
    auto maxZ = options.check("maxZ", yarp::os::Value(1.0f)).asFloat32();
    auto minX = options.check("minX", yarp::os::Value(-1.0f)).asFloat32();
    auto minY = options.check("minY", yarp::os::Value(-1.0f)).asFloat32();
    auto minZ = options.check("minZ", yarp::os::Value(-1.0f)).asFloat32();
    auto negative = options.check("negative", yarp::os::Value(false)).asBool();
    auto rotationX = options.check("rotationX", yarp::os::Value(0.0f)).asFloat32();
    auto rotationY = options.check("rotationY", yarp::os::Value(0.0f)).asFloat32();
    auto rotationZ = options.check("rotationZ", yarp::os::Value(0.0f)).asFloat32();
    auto translationX = options.check("translationX", yarp::os::Value(0.0f)).asFloat32();
    auto translationY = options.check("translationY", yarp::os::Value(0.0f)).asFloat32();
    auto translationZ = options.check("translationZ", yarp::os::Value(0.0f)).asFloat32();

    pcl::CropBox<T> cropper;
    cropper.setInputCloud(in);
    cropper.setKeepOrganized(keepOrganized);
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
void doGridMinimum(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto resolution = options.check("resolution", yarp::os::Value(0.0f)).asFloat32();
    pcl::GridMinimum<T> grid(resolution);
    grid.setInputCloud(in);
    grid.filter(*out);
    checkOutput<T>(out, "GridMinimum");
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
void doLocalMaximum(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto negative = options.check("negative", yarp::os::Value(false)).asBool();
    auto radius = options.check("radius", yarp::os::Value(1.0f)).asFloat32();

    pcl::LocalMaximum<T> local;
    local.setInputCloud(in);
    local.setNegative(negative);
    local.setRadius(radius);
    local.filter(*out);

    checkOutput<T>(out, "LocalMaximum");
}

template <typename T>
void doMarchingCubesHoppe(const typename pcl::PointCloud<T>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
{
#if PCL_VERSION_COMPARE(>=, 1, 9, 0)
    auto distanceIgnore = options.check("distanceIgnore", yarp::os::Value(-1.0f)).asFloat32();
#endif
    auto gridResolution = options.check("gridResolution", yarp::os::Value(32)).asInt32();
    auto gridResolutionX = options.check("gridResolutionX", yarp::os::Value(gridResolution)).asInt32();
    auto gridResolutionY = options.check("gridResolutionY", yarp::os::Value(gridResolution)).asInt32();
    auto gridResolutionZ = options.check("gridResolutionZ", yarp::os::Value(gridResolution)).asInt32();
    auto isoLevel = options.check("isoLevel", yarp::os::Value(0.0f)).asFloat32();
    auto percentageExtendGrid = options.check("percentageExtendGrid", yarp::os::Value(0.0f)).asFloat32();

    typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
    tree->setInputCloud(in);

    pcl::MarchingCubesHoppe<T> hoppe;
#if PCL_VERSION_COMPARE(>=, 1, 9, 0)
    hoppe.setDistanceIgnore(distanceIgnore);
#endif
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

template <typename T>
void doMedianFilter(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto maxAllowedMovement = options.check("maxAllowedMovement", yarp::os::Value(std::numeric_limits<float>::max())).asFloat32();
    auto windowSize = options.check("windowSize", yarp::os::Value(5)).asInt32();

    pcl::MedianFilter<T> median;
    median.setInputCloud(in);
    median.setMaxAllowedMovement(maxAllowedMovement);
    median.setWindowSize(windowSize);
    median.filter(*out);

    checkOutput<T>(out, "MedianFilter");
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

#if PCL_VERSION_COMPARE(>=, 1, 9, 0)
template <typename T1, typename T2 = T1>
void doMovingLeastSquares(const typename pcl::PointCloud<T1>::ConstPtr & in, const typename pcl::PointCloud<T2>::Ptr & out, const yarp::os::Searchable & options)
{
    auto cacheMlsResults = options.check("cacheMlsResults", yarp::os::Value(true)).asBool();
    auto computeNormals = options.check("computeNormals", yarp::os::Value(false)).asBool();
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

    typename pcl::MovingLeastSquares<T1, T2>::UpsamplingMethod upsamplingMethod;

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

    typename pcl::search::KdTree<T1>::Ptr tree(new pcl::search::KdTree<T1>());
    tree->setInputCloud(in);

    pcl::MovingLeastSquares<T1, T2> mls;
    mls.setCacheMLSResults(cacheMlsResults);
    mls.setComputeNormals(computeNormals);
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

    checkOutput<T2>(out, "MovingLeastSquares");
}
#endif

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
        // the implementation lacks a proper check
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
void doPassThrough(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto filterFieldName = options.check("filterFieldName", yarp::os::Value("")).asString();
    auto filterLimitMax = options.check("filterLimitMax", yarp::os::Value(std::numeric_limits<float>::max())).asFloat32();
    auto filterLimitMin = options.check("filterLimitMin", yarp::os::Value(std::numeric_limits<float>::min())).asFloat32();
    auto negative = options.check("negative", yarp::os::Value(false)).asBool();

    pcl::PassThrough<T> pass;
    pass.setFilterFieldName(filterFieldName);
    pass.setFilterLimits(filterLimitMin, filterLimitMax);
    pass.setInputCloud(in);
    pass.setNegative(negative);
    pass.filter(*out);

    checkOutput<T>(out, "PassThrough");
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

template <typename T>
void doRadiusOutlierRemoval(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto minNeighborsInRadius = options.check("minNeighborsInRadius", yarp::os::Value(1)).asInt32();
    auto negative = options.check("negative", yarp::os::Value(false)).asBool();
    auto radiusSearch = options.check("radiusSearch", yarp::os::Value(0.0)).asFloat64();

    pcl::RadiusOutlierRemoval<T> remover;
    remover.setInputCloud(in);
    remover.setMinNeighborsInRadius(minNeighborsInRadius);
    remover.setNegative(negative);
    remover.setRadiusSearch(radiusSearch);
    remover.filter(*out);

    checkOutput<T>(out, "RadiusOutlierRemoval");
}

template <typename T>
void doRandomSample(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto negative = options.check("negative", yarp::os::Value(false)).asBool();
    auto sample = options.check("sample", yarp::os::Value(std::numeric_limits<int>::max())).asInt64(); // note the shortening conversion
    auto seed = options.check("seed", yarp::os::Value(static_cast<int>(std::time(nullptr)))).asInt64(); // note the shortening conversion

    pcl::RandomSample<T> random;
    random.setInputCloud(in);
    random.setNegative(negative);
    random.setSample(sample);
    random.setSeed(seed);
    random.filter(*out);

    checkOutput<T>(out, "RandomSample");
}

template <typename T>
void doSamplingSurfaceNormal(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto ratio = options.check("ratio", yarp::os::Value(0.0f)).asFloat32();
    auto sample = options.check("sample", yarp::os::Value(10)).asInt32();
    auto seed = options.check("seed", yarp::os::Value(static_cast<int>(std::time(nullptr)))).asInt64(); // note the shortening conversion

    pcl::SamplingSurfaceNormal<T> sampler;
    sampler.setInputCloud(in);
    sampler.setRatio(ratio);
    sampler.setSample(sample);
    sampler.setSeed(seed);
    sampler.filter(*out);

    checkOutput<T>(out, "SamplingSurfaceNormal");
}

template <typename T>
void doShadowPoints(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto keepOrganized = options.check("keepOrganized", yarp::os::Value(false)).asBool();
    auto negative = options.check("negative", yarp::os::Value(false)).asBool();
    auto threshold = options.check("threshold", yarp::os::Value(0.1f)).asFloat32();

    typename pcl::PointCloud<T>::Ptr temp = boost::const_pointer_cast<pcl::PointCloud<T>>(in); // cast away constness

    pcl::ShadowPoints<T, T> shadow;
    shadow.setInputCloud(in);
    shadow.setKeepOrganized(keepOrganized);
    shadow.setNegative(negative);
    shadow.setNormals(temp); // assumes normals are contained in the input cloud
    shadow.setThreshold(threshold);
    shadow.filter(*out);

    checkOutput<T>(out, "ShadowPoints");
}

void doSimplificationRemoveUnusedVertices(const pcl::PolygonMesh::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
{
    pcl::surface::SimplificationRemoveUnusedVertices cleaner;
    cleaner.simplify(*in, *out);
    checkOutput(out, "doSimplificationRemoveUnusedVertices");
}

template <typename T>
void doStatisticalOutlierRemoval(const typename pcl::PointCloud<T>::ConstPtr & in, const typename pcl::PointCloud<T>::Ptr & out, const yarp::os::Searchable & options)
{
    auto meanK = options.check("meanK", yarp::os::Value(1)).asInt32();
    auto negative = options.check("negative", yarp::os::Value(false)).asBool();
    auto stddevMulThresh = options.check("stddevMulThresh", yarp::os::Value(0.0)).asFloat64();

    pcl::StatisticalOutlierRemoval<T> remover;
    remover.setInputCloud(in);
    remover.setMeanK(meanK);
    remover.setNegative(negative);
    remover.setStddevMulThresh(stddevMulThresh);
    remover.filter(*out);

    checkOutput<T>(out, "StatisticalOutlierRemoval");
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
    auto limitMax = options.check("limitMax", yarp::os::Value(std::numeric_limits<float>::max())).asFloat64();
    auto limitMin = options.check("limitMin", yarp::os::Value(-std::numeric_limits<float>::max())).asFloat64();
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

} // namespace

#endif // __YARP_CLOUD_UTILS_PCL_IMPL_HPP__
