#include "YarpCloudUtils.hpp"

#include <yarp/os/LogStream.h>
#ifdef HAVE_PCL
#include <yarp/pcl/Pcl.h>

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>
#include <exception>

#include <pcl/pcl_config.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
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
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>

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

    void crop(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & in, const pcl::PointCloud<pcl::PointXYZ>::Ptr & out, const yarp::os::Searchable & options)
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

        pcl::CropBox<pcl::PointXYZ> cropper;
        cropper.setInputCloud(in);
        cropper.setKeepOrganized(true); // don't remove points from the cloud, fill holes with NaNs
        cropper.setMax({maxX, maxY, maxZ, 1.0f});
        cropper.setMin({minX, minY, minZ, 1.0f});
        cropper.setNegative(negative);
        cropper.setRotation({rotationX, rotationY, rotationZ});
        cropper.setTranslation({translationX, translationY, translationZ});
        cropper.filter(*out);
    }

    void downsample(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & in, const pcl::PointCloud<pcl::PointXYZ>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto leafSize = options.check("downsampleLeafSize", yarp::os::Value(0.02f)).asFloat32();
        auto leafSizeX = options.check("downsampleLeafSizeX", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeY = options.check("downsampleLeafSizeY", yarp::os::Value(leafSize)).asFloat32();
        auto leafSizeZ = options.check("downsampleLeafSizeZ", yarp::os::Value(leafSize)).asFloat32();

        pcl::Filter<pcl::PointXYZ>::Ptr downsampler;

        if (options.check("downsampleUseApproximate", yarp::os::Value(false)).asBool())
        {
            auto * grid = new pcl::ApproximateVoxelGrid<pcl::PointXYZ>();
            grid->setDownsampleAllData(false); // just XYZ data
            grid->setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
            downsampler.reset(grid);
        }
        else
        {
            auto limitMax = options.check("downsampleLimitMax", yarp::os::Value(FLT_MAX)).asFloat64();
            auto limitMin = options.check("downsampleLimitMin", yarp::os::Value(-FLT_MAX)).asFloat64();
            auto limitsNegative = options.check("downsampleLimitsNegative", yarp::os::Value(false)).asBool();
            auto minimumPointsNumberPerVoxel = options.check("downsampleMinimumPointsNumberPerVoxel", yarp::os::Value(0)).asInt32();

            auto * grid = new pcl::VoxelGrid<pcl::PointXYZ>();
            grid->setDownsampleAllData(false); // just XYZ data
            grid->setFilterLimits(limitMin, limitMax);
            grid->setFilterLimitsNegative(limitsNegative);
            grid->setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
            grid->setMinimumPointsNumberPerVoxel(minimumPointsNumberPerVoxel);
            grid->setSaveLeafLayout(false);
            downsampler.reset(grid);
        }

        downsampler->setInputCloud(in);
        downsampler->filter(*out);
    }

    void smooth(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & in, const pcl::PointCloud<pcl::PointXYZ>::Ptr & out, const yarp::os::Searchable & options)
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

        if (projectionMethodStr == "simple")
        {
            projectionMethod = pcl::MLSResult::ProjectionMethod::SIMPLE;
        }
        else if (projectionMethodStr == "orthogonal")
        {
            projectionMethod = pcl::MLSResult::ProjectionMethod::ORTHOGONAL;
        }
        else
        {
            if (projectionMethodStr != "none")
            {
                yWarning() << "unknown projection method" << projectionMethodStr << "for smooth step, falling back to none";
            }

            projectionMethod = pcl::MLSResult::ProjectionMethod::NONE;
        }

        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::UpsamplingMethod upsamplingMethod;

        if (upsamplingMethodStr == "distinctCloud")
        {
            upsamplingMethod = decltype(upsamplingMethod)::DISTINCT_CLOUD;
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
            if (upsamplingMethodStr != "none")
            {
                yWarning() << "unknown upsampling method" << upsamplingMethodStr << "for smooth step, falling back to none";
            }

            upsamplingMethod = decltype(upsamplingMethod)::NONE;
        }

        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(in);
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
    }

    void estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & in, const pcl::PointCloud<pcl::Normal>::Ptr & out, const yarp::os::Searchable & options)
    {
        // Either radius search or nearest K search; one of these params must be zero.
        auto kSearch = options.check("estimatorKSearch", yarp::os::Value(40)).asInt32();
        auto radiusSearch = options.check("estimatorRadiusSearch", yarp::os::Value(0.0)).asFloat64();

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr estimator;

        if (options.check("estimatorUseOMP", yarp::os::Value(true)).asBool())
        {
            auto numberOfThreads = options.check("estimatorNumberOfThreads", yarp::os::Value(0)).asInt32();
            auto * omp = new pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>();
            omp->setNumberOfThreads(numberOfThreads);
            estimator.reset(omp);
        }
        else
        {
            estimator.reset(new pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>());
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(in);
        estimator->setInputCloud(in);
        estimator->setKSearch(kSearch);
        estimator->setRadiusSearch(radiusSearch);
        estimator->setSearchMethod(tree);
        estimator->compute(*out);
    }

    void reconstruct(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        auto method = options.find("surfaceMethod").asString();

        pcl::PCLSurfaceBase<pcl::PointXYZ>::Ptr surface;

        if (method == "concave")
        {
            auto alpha = options.check("surfaceAlpha", yarp::os::Value(0.0)).asFloat64();
            auto * concave = new pcl::ConcaveHull<pcl::PointXYZ>();
            concave->setAlpha(alpha);
            concave->setDimension(3);
            concave->setKeepInformation(false); // discard RGB, normals...
            surface.reset(concave);
        }
        else if (method == "convex")
        {
            auto * convex = new pcl::ConvexHull<pcl::PointXYZ>();
            convex->setComputeAreaVolume(false);
            convex->setDimension(3);
            surface.reset(convex);
        }
        else if (method == "organized")
        {
            if (!in->isOrganized())
            {
                yWarning() << "input cloud must be organized (height > 1)";
                return;
            }

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
            auto triangulationTypeStr = options.check("surfaceTriangulationType", yarp::os::Value("triangleAdaptiveCut")).asString();
            auto useDepthAsDistance = options.check("surfaceUseDepthAsDistance", yarp::os::Value(false)).asBool();

            pcl::OrganizedFastMesh<pcl::PointXYZ>::TriangulationType triangulationType;

            if (triangulationTypeStr == "quadMesh")
            {
                triangulationType = decltype(triangulationType)::QUAD_MESH;
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
                if (triangulationTypeStr != "triangleAdaptiveCut")
                {
                    yWarning() << "unknown triangulation type" << triangulationTypeStr << "for reconstruct step, falling back to triangleAdaptiveCut";
                }

                triangulationType = decltype(triangulationType)::TRIANGLE_ADAPTIVE_CUT;
            }

            auto * organized = new pcl::OrganizedFastMesh<pcl::PointXYZ>();
            organized->setAngleTolerance(angleTolerance);
            organized->setDistanceTolerance(distanceTolerance, depthDependent);
            organized->setMaxEdgeLength(maxEdgeLengthA, maxEdgeLengthB, maxEdgeLengthC);
            organized->setTrianglePixelSize(trianglePixelSize);
            organized->setTrianglePixelSizeColumns(trianglePixelSizeColumns);
            organized->setTrianglePixelSizeRows(trianglePixelSizeRows);
            organized->setTriangulationType(triangulationType);
            organized->storeShadowedFaces(storeShadowedFaces);
            organized->useDepthAsDistance(useDepthAsDistance);
            surface.reset(organized);
        }
        else
        {
            // we should not be here
            yFatal() << "unrecognized surface method:" << method;
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(in);
        surface->setInputCloud(in);
        surface->setSearchMethod(tree);
        surface->reconstruct(*out);
    }

    void reconstruct(const pcl::PointCloud<pcl::PointNormal>::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        const auto fallback = "poisson";
        auto method = options.check("surfaceMethod", yarp::os::Value(fallback)).asString();

        pcl::PCLSurfaceBase<pcl::PointNormal>::Ptr surface;

        if (method == "gp")
        {
            auto maxBinarySearchLevel = options.check("surfaceMaxBinarySearchLevel", yarp::os::Value(10)).asInt32();
            auto nearestNeighborNum = options.check("surfaceNearestNeighborNum", yarp::os::Value(50)).asInt32();
            auto paddingSize = options.check("surfacePaddingSize", yarp::os::Value(3)).asInt32();
            auto resolution = options.check("surfaceResolution", yarp::os::Value(0.001)).asFloat64();

            auto * gp = new pcl::GridProjection<pcl::PointNormal>();
            gp->setMaxBinarySearchLevel(maxBinarySearchLevel);
            gp->setNearestNeighborNum(nearestNeighborNum);
            gp->setPaddingSize(paddingSize);
            gp->setResolution(resolution);
            surface.reset(gp);
        }
        else if (method == "gp3")
        {
            auto consistentVertexOrdering = options.check("surfaceConsistentVertexOrdering", yarp::os::Value(false)).asBool();
            auto maximumAngle = options.check("surfaceMaximumAngle", yarp::os::Value(2 * M_PI / 3)).asFloat64();
            auto maximumNearestNeighbors = options.check("surfaceMaximumNearestNeighbors", yarp::os::Value(100)).asInt32();
            auto maximumSurfaceAngle = options.check("surfaceMaximumSurfaceAngle", yarp::os::Value(M_PI / 4)).asFloat64();
            auto minimumAngle = options.check("surfaceMinimumAngle", yarp::os::Value(M_PI / 18)).asFloat64();
            auto mu = options.check("surfaceMu", yarp::os::Value(0.0)).asFloat64();
            auto searchRadius = options.check("surfaceSearchRadius", yarp::os::Value(0.0)).asFloat64();

            auto * gp3 = new pcl::GreedyProjectionTriangulation<pcl::PointNormal>();
            gp3->setConsistentVertexOrdering(consistentVertexOrdering);
            gp3->setMaximumAngle(maximumAngle);
            gp3->setMaximumSurfaceAngle(maximumSurfaceAngle);
            gp3->setMaximumNearestNeighbors(maximumNearestNeighbors);
            gp3->setMinimumAngle(minimumAngle);
            gp3->setMu(mu);
            gp3->setNormalConsistency(true); // input normals are oriented consistently
            gp3->setSearchRadius(searchRadius);
            surface.reset(gp3);
        }
        else if (method == "mchoppe")
        {
            auto distanceIgnore = options.check("surfaceDistanceIgnore", yarp::os::Value(-1.0f)).asFloat32();
            auto gridResolution = options.check("surfaceGridResolution", yarp::os::Value(32)).asInt32();
            auto gridResolutionX = options.check("surfaceGridResolutionX", yarp::os::Value(gridResolution)).asInt32();
            auto gridResolutionY = options.check("surfaceGridResolutionY", yarp::os::Value(gridResolution)).asInt32();
            auto gridResolutionZ = options.check("surfaceGridResolutionZ", yarp::os::Value(gridResolution)).asInt32();
            auto isoLevel = options.check("surfaceIsoLevel", yarp::os::Value(0.0f)).asFloat32();
            auto percentageExtendGrid = options.check("surfacePercentageExtendGrid", yarp::os::Value(0.0f)).asFloat32();

            auto * hoppe = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
            hoppe->setDistanceIgnore(distanceIgnore);
            hoppe->setGridResolution(gridResolutionX, gridResolutionY, gridResolutionZ);
            hoppe->setIsoLevel(isoLevel);
            hoppe->setPercentageExtendGrid(percentageExtendGrid);
            surface.reset(hoppe);
        }
        else if (method == "mcrbf")
        {
            auto gridResolution = options.check("surfaceGridResolution", yarp::os::Value(32)).asInt32();
            auto gridResolutionX = options.check("surfaceGridResolutionX", yarp::os::Value(gridResolution)).asInt32();
            auto gridResolutionY = options.check("surfaceGridResolutionY", yarp::os::Value(gridResolution)).asInt32();
            auto gridResolutionZ = options.check("surfaceGridResolutionZ", yarp::os::Value(gridResolution)).asInt32();
            auto isoLevel = options.check("surfaceIsoLevel", yarp::os::Value(0.0f)).asFloat32();
            auto offSurfaceDisplacement = options.check("surfaceOffSurfaceDisplacement", yarp::os::Value(0.1f)).asFloat32();
            auto percentageExtendGrid = options.check("surfacePercentageExtendGrid", yarp::os::Value(0.0f)).asFloat32();

            auto * rbf = new pcl::MarchingCubesRBF<pcl::PointNormal>();
            rbf->setGridResolution(gridResolutionX, gridResolutionY, gridResolutionZ);
            rbf->setIsoLevel(isoLevel);
            rbf->setOffSurfaceDisplacement(offSurfaceDisplacement);
            rbf->setPercentageExtendGrid(percentageExtendGrid);
            surface.reset(rbf);
        }
        else if (method == "poisson")
        {
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

            auto * poisson = new pcl::Poisson<pcl::PointNormal>();
            poisson->setConfidence(true); // all normals are presumed normalized to have unit-length prior to reconstruction
            poisson->setDegree(degree);
            poisson->setDepth(depth);
            poisson->setIsoDivide(isoDivide);
            poisson->setManifold(manifold);
            poisson->setMinDepth(minDepth);
            poisson->setOutputPolygons(outputPolygons);
            poisson->setPointWeight(pointWeight);
            poisson->setSamplesPerNode(samplesPerNode);
            poisson->setScale(scale);
            poisson->setSolverDivide(solverDivide);
#if PCL_VERSION_COMPARE(>=, 1, 12, 0)
            poisson->setThreads(threads);
#endif
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

    void process(const pcl::PolygonMesh::ConstPtr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        const auto fallback = "laplacian";
        auto method = options.check("processMethod", yarp::os::Value(fallback)).asString();

        pcl::MeshProcessing::Ptr processor;

        if (method == "laplacian")
        {
            auto boundarySmoothing = options.check("processBoundarySmoothing", yarp::os::Value(true)).asBool();
            auto convergence = options.check("processConvergence", yarp::os::Value(0.0f)).asFloat32();
            auto edgeAngle = options.check("processEdgeAngle", yarp::os::Value(15.0f)).asFloat32();
            auto featureAngle = options.check("processFeatureAngle", yarp::os::Value(45.0f)).asFloat32();
            auto featureEdgeSmoothing = options.check("processFeatureEdgeSmoothing", yarp::os::Value(false)).asBool();
            auto numIter = options.check("processNumIter", yarp::os::Value(20)).asInt32();
            auto relaxationFactor = options.check("processRelaxationFactor", yarp::os::Value(0.01f)).asFloat32();

            auto * laplacian = new pcl::MeshSmoothingLaplacianVTK();
            laplacian->setBoundarySmoothing(boundarySmoothing);
            laplacian->setConvergence(convergence);
            laplacian->setEdgeAngle(edgeAngle);
            laplacian->setFeatureAngle(featureAngle);
            laplacian->setFeatureEdgeSmoothing(featureEdgeSmoothing);
            laplacian->setNumIter(numIter);
            laplacian->setRelaxationFactor(relaxationFactor);
            processor.reset(laplacian);
        }
        else if (method == "quadric")
        {
            auto targetReductionFactor = options.check("processTargetReductionFactor", yarp::os::Value(0.5f)).asFloat32();
            auto * quadric = new pcl::MeshQuadricDecimationVTK();
            quadric->setTargetReductionFactor(targetReductionFactor);
            processor.reset(quadric);
        }
        else if (method == "subdivision")
        {
            auto filterTypeStr = options.check("processFilterType", yarp::os::Value("linear")).asString();

            pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType filterType;

            if (filterTypeStr == "butterfly")
            {
                filterType = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::BUTTERFLY;
            }
            else if (filterTypeStr == "loop")
            {
                filterType = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::LOOP;
            }
            else
            {
                if (filterTypeStr != "linear")
                {
                    yWarning() << "unknown filter type" << filterTypeStr << "for process step, falling back to linear";
                }

                filterType = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::LINEAR;
            }

            auto * subdivision = new pcl::MeshSubdivisionVTK();
            subdivision->setFilterType(filterType);
            processor.reset(subdivision);
        }
        else if (method == "windowed")
        {
            auto boundarySmoothing = options.check("processBoundarySmoothing", yarp::os::Value(true)).asBool();
            auto edgeAngle = options.check("processEdgeAngle", yarp::os::Value(15.0f)).asFloat32();
            auto featureAngle = options.check("processFeatureAngle", yarp::os::Value(45.0f)).asFloat32();
            auto featureEdgeSmoothing = options.check("processFeatureEdgeSmoothing", yarp::os::Value(false)).asBool();
            auto normalizeCoordinates = options.check("processNormalizeCoordinates", yarp::os::Value(false)).asBool();
            auto numIter = options.check("processNumIter", yarp::os::Value(20)).asInt32();
            auto passBand = options.check("processPassBand", yarp::os::Value(0.1f)).asFloat32();

            auto * windowed = new pcl::MeshSmoothingWindowedSincVTK();
            windowed->setBoundarySmoothing(boundarySmoothing);
            windowed->setEdgeAngle(edgeAngle);
            windowed->setFeatureAngle(featureAngle);
            windowed->setFeatureEdgeSmoothing(featureEdgeSmoothing);
            windowed->setNormalizeCoordinates(normalizeCoordinates);
            windowed->setNumIter(numIter);
            windowed->setPassBand(passBand);
            processor.reset(windowed);
        }
        else
        {
            yWarning() << "unrecognized process method:" << method;
            yInfo() << "falling back with default parameters to" << fallback;
        }

        processor->setInputMesh(in);
        processor->process(*out);
    }

    void meshFromCloudPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PolygonMesh::Ptr & mesh, const yarp::os::Searchable & options)
    {
        // Crop input cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped = cloud;

        if (!options.check("cropSkip", yarp::os::Value(true)).asBool())
        {
            cropped.reset(new pcl::PointCloud<pcl::PointXYZ>());
            crop(cloud, cropped, options);
        }

        // Downsample so that further computations are actually feasible.
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = cropped;

        if (!options.check("downsampleSkip", yarp::os::Value(false)).asBool())
        {
            filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
            downsample(cropped, filtered, options);
        }

        // Pre-process input cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed = filtered;

        if (!options.check("smoothSkip", yarp::os::Value(true)).asBool())
        {
            smoothed.reset(new pcl::PointCloud<pcl::PointXYZ>());
            smooth(filtered, smoothed, options);
        }

        // Reconstruct triangle mesh.
        pcl::PolygonMesh::Ptr reconstructed(new pcl::PolygonMesh());
        auto method = options.check("surfaceMethod", yarp::os::Value("")).asString();

        if (method == "concave" || method == "convex" || method == "organized")
        {
            reconstruct(smoothed, reconstructed, options);
        }
        else
        {
            // Estimate normals.
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
            estimateNormals(smoothed, normals, options);

            // Concatenate point clouds.
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
            pcl::concatenateFields(*filtered, *normals, *cloudWithNormals);

            reconstruct(cloudWithNormals, reconstructed, options);
        }

        // Post-process output mesh.
        pcl::PolygonMesh::Ptr processed = reconstructed;

        if (!options.check("processSkip", yarp::os::Value(true)).asBool())
        {
            processed.reset(new pcl::PolygonMesh());
            process(reconstructed, processed, options);
        }

        mesh = processed;
    }
}
#endif

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T>
bool meshFromCloud(const yarp::sig::PointCloud<T> & cloud, yarp::sig::PointCloudXYZ & meshPoints, yarp::sig::VectorOf<int> & meshIndices, const yarp::os::Searchable & options)
{
#ifdef HAVE_PCL
    // Get a XYZ point cloud.
    yarp::sig::PointCloudXYZ cloudXYZ(cloud);

    // Convert YARP cloud to PCL cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclXYZ(new pcl::PointCloud<pcl::PointXYZ>());
    yarp::pcl::toPCL(cloudXYZ, *pclXYZ);

    // Perform surface reconstruction.
    pcl::PolygonMesh::Ptr pclMesh(new pcl::PolygonMesh());

    try
    {
        meshFromCloudPCL(pclXYZ, pclMesh, options);
    }
    catch (const std::exception & e)
    {
        yError() << "meshFromCloudPCL:" << e.what();
        return false;
    }

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
