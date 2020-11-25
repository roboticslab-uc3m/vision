#include "YarpCloudUtils.hpp"

#include <yarp/os/LogStream.h>
#ifdef HAVE_PCL
#include <yarp/pcl/Pcl.h>

#include <cfloat>
#include <cmath>

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
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/organized_fast_mesh.h>
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

    void smooth(const pcl::PointCloud<pcl::PointXYZ>::Ptr & in, const pcl::PointCloud<pcl::PointXYZ>::Ptr & out, const yarp::os::Searchable & options)
    {
        auto order = options.check("smoothOrder", yarp::os::Value(2)).asInt32();
        auto projectionMethodStr = options.check("smoothProjectionMethod", yarp::os::Value("simple")).asString();
        auto searchRadius = options.check("smoothSearchRadius", yarp::os::Value(0.0)).asFloat64();
        auto sqrGaussParam = options.check("smoothSqrGaussParam", yarp::os::Value(0.0)).asFloat64();
        auto threads = options.check("smoothThreads", yarp::os::Value(1)).asInt32();

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

        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(in);
        mls.setInputCloud(in);
        mls.setCacheMLSResults(true); // TODO
        mls.setComputeNormals(false);
        mls.setNumberOfThreads(threads);
        mls.setPolynomialOrder(order);
        mls.setProjectionMethod(projectionMethod);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(searchRadius);
        mls.setSqrGaussParam(sqrGaussParam);
        mls.setUpsamplingMethod(decltype(mls)::UpsamplingMethod::NONE);
        mls.process(*out);
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

        if (method == "concave")
        {
            auto alpha = options.check("surfaceAlpha", yarp::os::Value(0.0)).asFloat64();
            auto * concave = new pcl::ConcaveHull<pcl::PointNormal>();
            concave->setAlpha(alpha);
            concave->setDimension(3);
            concave->setKeepInformation(false);
            surface.reset(concave);
        }
        else if (method == "convex")
        {
            auto * convex = new pcl::ConvexHull<pcl::PointNormal>();
            convex->setComputeAreaVolume(false);
            convex->setDimension(3);
            surface.reset(convex);
        }
        else if (method == "gp")
        {
            auto k = options.check("surfaceK", yarp::os::Value(50)).asInt32();
            auto maxBinarySearchLevel = options.check("surfaceMaxBinarySearchLevel", yarp::os::Value(10)).asInt32();
            auto paddingSize = options.check("surfacePaddingSize", yarp::os::Value(3)).asInt32();
            auto resolution = options.check("surfaceResolution", yarp::os::Value(0.001)).asFloat64();

            auto * gp = new pcl::GridProjection<pcl::PointNormal>();
            gp->setMaxBinarySearchLevel(maxBinarySearchLevel);
            gp->setNearestNeighborNum(k);
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
        else if (method == "organized")
        {
            auto angleTolerance = options.check("surfaceAngleTolerance", yarp::os::Value(12.5 * M_PI / 180)).asFloat32();
            auto depthDependent = options.check("surfaceDepthDependent", yarp::os::Value(false)).asBool();
            auto distanceTolerance = options.check("surfaceDistanceTolerance", yarp::os::Value(-1.0f)).asFloat32();
            auto maxEdgeLengthA = options.check("surfaceMaxEdgeLengthA", yarp::os::Value(0.0f)).asFloat32();
            auto maxEdgeLengthB = options.check("surfaceMaxEdgeLengthB", yarp::os::Value(0.0f)).asFloat32();
            auto maxEdgeLengthC = options.check("surfaceMaxEdgeLengthC", yarp::os::Value(0.0f)).asFloat32();
            auto trianglePixelSize = options.check("surfaceTrianglePixelSize", yarp::os::Value(1)).asInt32();
            auto trianglePixelSizeColumns = options.check("surfaceTrianglePixelSizeColumns", yarp::os::Value(trianglePixelSize)).asInt32();
            auto trianglePixelSizeRows = options.check("surfaceTrianglePixelSizeRows", yarp::os::Value(trianglePixelSize)).asInt32();
            auto triangulationTypeStr = options.check("surfaceTriangulationType", yarp::os::Value("quad")).asString();
            auto useDepthAsDistance = options.check("surfaceUseDepthAsDistance", yarp::os::Value(false)).asBool();

            pcl::OrganizedFastMesh<pcl::PointNormal>::TriangulationType triangulationType;

            if (triangulationTypeStr == "adaptive")
            {
                triangulationType = pcl::OrganizedFastMesh<pcl::PointNormal>::TriangulationType::TRIANGLE_ADAPTIVE_CUT;
            }
            else if (triangulationTypeStr == "left")
            {
                triangulationType = pcl::OrganizedFastMesh<pcl::PointNormal>::TriangulationType::TRIANGLE_LEFT_CUT;
            }
            else if (triangulationTypeStr == "right")
            {
                triangulationType = pcl::OrganizedFastMesh<pcl::PointNormal>::TriangulationType::TRIANGLE_RIGHT_CUT;
            }
            else
            {
                if (triangulationTypeStr != "quad")
                {
                    yWarning() << "unknown triangulation type" << triangulationTypeStr << "for process step, falling back to quad";
                }

                triangulationType = pcl::OrganizedFastMesh<pcl::PointNormal>::TriangulationType::QUAD_MESH;
            }

            auto * organized = new pcl::OrganizedFastMesh<pcl::PointNormal>();
            organized->setAngleTolerance(angleTolerance);
            organized->setDistanceTolerance(distanceTolerance, depthDependent);
            organized->setMaxEdgeLength(maxEdgeLengthA, maxEdgeLengthB, maxEdgeLengthC);
            organized->setTrianglePixelSize(trianglePixelSize);
            organized->setTrianglePixelSizeColumns(trianglePixelSizeColumns);
            organized->setTrianglePixelSizeRows(trianglePixelSizeRows);
            organized->setTriangulationType(triangulationType);
            organized->storeShadowedFaces(false);
            organized->useDepthAsDistance(useDepthAsDistance);
            surface.reset(organized);
        }
        else if (method == "poisson")
        {
            auto degree = options.check("surfaceDegree", yarp::os::Value(2)).asInt32();
            auto depth = options.check("surfaceDepth", yarp::os::Value(8)).asInt32();
            auto isoDivide = options.check("surfaceIsoDivide", yarp::os::Value(8)).asInt32();
            auto manifold = options.check("surfaceManifold", yarp::os::Value(true)).asBool(); // TODO
            auto minDepth = options.check("surfaceMinDepth", yarp::os::Value(5)).asInt32();
            auto outputPolygons = options.check("surfaceOutputPolygons", yarp::os::Value(false)).asBool(); // TODO
            auto pointWeight = options.check("surfacePointWeight", yarp::os::Value(4.0f)).asFloat32();
            auto samplesPerNode = options.check("surfaceSamplesPerNode", yarp::os::Value(1.0f)).asFloat32();
            auto scale = options.check("surfaceScale", yarp::os::Value(1.1f)).asFloat32();
            auto solverDivide = options.check("surfaceSolverDivide", yarp::os::Value(8)).asInt32();

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

    void process(const pcl::PolygonMesh::Ptr & in, const pcl::PolygonMesh::Ptr & out, const yarp::os::Searchable & options)
    {
        const auto fallback = "laplacian";
        auto method = options.check("processMethod", yarp::os::Value(fallback)).asString();

        if (method == "ear")
        {}
        else if (method == "laplacian")
        {}
        else if (method == "quadric")
        {}
        else if (method == "subdivision")
        {}
        else if (method == "windowed")
        {}
        else
        {
            yWarning() << "unrecognized process method:" << method;
            yInfo() << "falling back with default parameters to" << fallback;
        }

        pcl::MeshProcessing::Ptr processor;
        processor->setInputMesh(in);
        processor->process(*out);
    }

    void meshFromCloudPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PolygonMesh::Ptr & mesh, const yarp::os::Searchable & options)
    {
        // Downsample so that further computations are actually feasible.
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = cloud;

        if (!options.check("filterSkip", yarp::os::Value(false)).asBool())
        {
            filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
            downsample(cloud, filtered, options);
        }

        // Pre-process input cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed = filtered;

        if (!options.check("smoothSkip", yarp::os::Value(true)).asBool())
        {
            smoothed.reset(new pcl::PointCloud<pcl::PointXYZ>());
            smooth(filtered, smoothed, options);
        }

        // Estimate normals.
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        estimateNormals(smoothed, normals, options);

        // Concatenate point clouds.
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
        pcl::concatenateFields(*filtered, *normals, *cloudWithNormals);

        // Reconstruct triangle mesh.
        pcl::PolygonMesh::Ptr reconstructed;
        reconstruct(cloudWithNormals, reconstructed, options);

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
