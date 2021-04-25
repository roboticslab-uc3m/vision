# YarpCloudUtils: point cloud utilities for YARP {#yarpcloudutils}

This library contains a collection of free functions that aim to provide support
for the family of `yarp::sig::PointCloud<T>` template specializations. For the time
being, `roboticslab::YarpCloudUtils` allows users to:

* save a point cloud or triangular polygon mesh to .ply file,
* read a point cloud or triangular polygon mesh from .ply file,
* construct a triangular polygon mesh from a point cloud, or
* process a point cloud to obtain another cloud.

Supported data types are:

* `yarp::sig::DataXY` (only for data import/export via .ply)
* `yarp::sig::DataXYZ`
* `yarp::sig::DataNormal` (only for data import/export via .ply)
* `yarp::sig::DataXYZRGBA`
* `yarp::sig::DataXYZI`
* `yarp::sig::DataInterestPointXYZ`
* `yarp::sig::DataXYZNormal`
* `yarp::sig::DataXYZNormalRGBA`

XY and plain-normal types are not available for surface meshing and cloud processing.

## Supported algorithms

Both `roboticslab::YarpCloudUtils::meshFromCloud` and `roboticslab::YarpCloudUtils::processCloud`
implement a set of [Point Cloud Library (PCL)](https://pointclouds.org/) algorithms.
A variety of PCL classes are exposed in a highly configurable pipeline:

| PCL class                                                                                                                                             | usage                  | point types               | notes                                                              |
|-------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------|---------------------------|--------------------------------------------------------------------|
| [`pcl::transformPointCloud`](https://pointclouds.org/documentation/group__common.html#gac841d05d13c925f3a3a8090d9d7ff24d)                             | affine transformation  | any                       | `translation` (meters) and `rotation` (scaled axis-angle, radians) |
| [`pcl::transformPointCloudWithNormals`](https://pointclouds.org/documentation/group__common.html#ga01dcf9e24dec3109a0c8a8b8f2e24bcc)                  | affine transformation  | any normal type           | see above                                                          |
| [`pcl::ApproximateVoxelGrid`](https://pointclouds.org/documentation/classpcl_1_1_approximate_voxel_grid.html)                                         | cloud resampling       | any                       |                                                                    |
| [`pcl::BilateralFilter`](https://pointclouds.org/documentation/classpcl_1_1_bilateral_filter.html)                                                    | cloud filtering        | XYZI(Normal)              |                                                                    |
| [`pcl::BilateralUpsampling`](https://pointclouds.org/documentation/classpcl_1_1_bilateral_upsampling.html)                                            | cloud processing       | XYZRGBA                   | organized clouds only                                              |
| [`pcl::ConcaveHull`](https://pointclouds.org/documentation/classpcl_1_1_concave_hull.html)                                                            | mesh construction      | any                       |                                                                    |
| [`pcl::ConvexHull`](https://pointclouds.org/documentation/classpcl_1_1_convex_hull.html)                                                              | mesh construction      | any                       |                                                                    |
| [`pcl::CropBox`](https://pointclouds.org/documentation/classpcl_1_1_crop_box.html)                                                                    | cloud filtering        | any                       |                                                                    |
| [`pcl::FastBilateralFilter`](https://pointclouds.org/documentation/classpcl_1_1_fast_bilateral_filter.html)                                           | cloud filtering        | XYZ(RGBA)                 | organized clouds only                                              |
| [`pcl::FastBilateralFilterOMP`](https://pointclouds.org/documentation/classpcl_1_1_fast_bilateral_filter_o_m_p.html)                                  | cloud filtering        | XYZ(RGBA)                 | organized clouds only                                              |
| [`pcl::GreedyProjectionTriangulation`](https://pointclouds.org/documentation/classpcl_1_1_greedy_projection_triangulation.html)                       | mesh construction      | XYZ/XYZI/XYZRGBA + Normal |                                                                    |
| [`pcl::GridMinimum`](https://pointclouds.org/documentation/classpcl_1_1_grid_minimum.html)                                                            | cloud resampling       | any                       |                                                                    |
| [ `pcl::GridProjection` ](https://pointclouds.org/documentation/classpcl_1_1_grid_projection.html)                                                    | surface reconstruction | XYZ/XYZI/XYZRGBA + Normal |                                                                    |
| [`pcl::LocalMaximum`](https://pointclouds.org/documentation/classpcl_1_1_local_maximum.html)                                                          | cloud resampling       | any                       |                                                                    |
| [`pcl::MarchingCubesHoppe`](https://pointclouds.org/documentation/classpcl_1_1_marching_cubes_hoppe.html)                                             | surface reconstruction | XYZ/XYZI/XYZRGBA + Normal |                                                                    |
| [`pcl::MarchingCubesRBF`](https://pointclouds.org/documentation/classpcl_1_1_marching_cubes_r_b_f.html)                                               | surface reconstruction | XYZ/XYZI/XYZRGBA + Normal |                                                                    |
| [`pcl::MedianFilter`](https://pointclouds.org/documentation/classpcl_1_1_median_filter.html)                                                          | cloud filtering        | any                       | organized clouds only                                              |
| [`pcl::MeshQuadricDecimationVTK`](https://pointclouds.org/documentation/classpcl_1_1_mesh_quadric_decimation_v_t_k.html)                              | mesh processing        | (mesh)                    |                                                                    |
| [`pcl::MeshSmoothingLaplacianVTK`](https://pointclouds.org/documentation/classpcl_1_1_mesh_smoothing_laplacian_v_t_k.html)                            | mesh processing        | (mesh)                    |                                                                    |
| [`pcl::MeshSmoothingWindowedSincVTK`](https://pointclouds.org/documentation/classpcl_1_1_mesh_smoothing_windowed_sinc_v_t_k.html)                     | mesh processing        | (mesh)                    |                                                                    |
| [`pcl::MeshSubdivisionVTK`](https://pointclouds.org/documentation/classpcl_1_1_mesh_subdivision_v_t_k.html)                                           | mesh processing        | (mesh)                    |                                                                    |
| [`pcl::MovingLeastSquares`](https://pointclouds.org/documentation/classpcl_1_1_moving_least_squares.html)                                             | cloud processing       | any                       | requires PCL 1.9+                                                  |
| [`pcl::NormalEstimation`](https://pointclouds.org/documentation/classpcl_1_1_normal_estimation.html)                                                  | normal estimation      | any                       |                                                                    |
| [`pcl::NormalEstimationOMP`](https://pointclouds.org/documentation/classpcl_1_1_normal_estimation_o_m_p.html)                                         | normal estimation      | any                       |                                                                    |
| [`pcl::OrganizedFastMesh`](https://pointclouds.org/documentation/classpcl_1_1_organized_fast_mesh.html)                                               | mesh construction      | XYZ(RGBA)                 | organized clouds only                                              |
| [`pcl::PassThrough`](https://pointclouds.org/documentation/classpcl_1_1_pass_through.html)                                                            | cloud filtering        | any                       |                                                                    |
| [`pcl::Poisson`](https://pointclouds.org/documentation/classpcl_1_1_poisson.html)                                                                     | surface reconstruction | XYZ/XYZI/XYZRGBA + Normal | `threads` requires PCL 1.12+                                       |
| [`pcl::RadiusOutlierRemoval`](https://pointclouds.org/documentation/classpcl_1_1_radius_outlier_removal.html)                                         | cloud filtering        | any                       |                                                                    |
| [`pcl::RandomSample`](https://pointclouds.org/documentation/classpcl_1_1_random_sample.html)                                                          | cloud resampling       | any                       |                                                                    |
| [`pcl::SamplingSurfaceNormal`](https://pointclouds.org/documentation/classpcl_1_1_sampling_surface_normal.html)                                       | cloud resampling       | XYZ/XYZI/XYZRGBA + Normal |                                                                    |
| [`pcl::ShadowPoints`](https://pointclouds.org/documentation/classpcl_1_1_shadow_points.html)                                                          | cloud filtering        | any normal type           | only normal types                                                  |
| [`pcl::SimplificationRemoveUnusedVertices`](https://pointclouds.org/documentation/classpcl_1_1surface_1_1_simplification_remove_unused_vertices.html) | mesh simplification    | (mesh)                    |                                                                    |
| [`pcl::StatisticalOutlierRemoval`](https://pointclouds.org/documentation/classpcl_1_1_statistical_outlier_removal.html)                               | cloud filtering        | any                       |                                                                    |
| [`pcl::UniformSampling`](https://pointclouds.org/documentation/classpcl_1_1_uniform_sampling.html)                                                    | cloud resampling       | any                       |                                                                    |
| [`pcl::VoxelGrid`](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html)                                                                | cloud resampling       | any                       |                                                                    |

Mesh construction methods preserve the original point cloud as the surface vertices
and simply construct the mesh on top, while surface reconstruction changes the topology
of the input cloud.

## Configuration

YARP mechanisms are taken advantage of to set up the pipeline, especially in terms
of the dictionary of key-values provided by `yarp::os::Property`.
Depending on the selected function overload, each step of the pipeline corresponds
either to an element of a vector of such dictionaries, or a group/dictionary within
a named section collection context.

As a rule of thumb, there will always be a key named `algorithm` which corresponds to
one of the supported PCL classes, i.e. the paired value must be equal to the class name
of the selected algorithm.

Remaining keys of the configured step, if any, correspond to configuration setters
pertaining to the algorithm, always following the camelCase naming convention.
For instance, in order to access the `pcl::Poisson::setMinDepth` setter, one would
create a dictionary key named `minDepth` and assign it an integer value.
In a similar manner, dictionary values are mapped to enumerations, e.g.
`pcl::OrganizedFastMesh::TriangulationType:QUAD_MESH` can be requested by assigning
`quadMesh` to key `triangulationType`.

In absence of key-value entries (beyond the mandatory `algorithm` one), the pipeline
assumes default values as defined by each class constructor.

**Note:** mind input/output types for each step. If the output of step N is not compatible
with the expected input of step N+1, an error will be reported.

### Vector of dictionaries

The most straightforward way to configure a pipeline is using the `VectorOf<Property>`
overload. Each element of the vector is one step of the pipeline, order is preserved:

```cxx
yarp::sig::VectorOf<yarp::os::Property> options {
    {
        {"algorithm", yarp::os::Value("VoxelGrid")},
        {"leafSize", yarp::os::Value(0.02f)}
    },
    {
        {"algorithm", yarp::os::Value("NormalEstimationOMP")},
        {"kSearch", yarp::os::Value(40)}
    },
    {
        {"algorithm", yarp::os::Value("Poisson")}
    }
};

bool ret = roboticslab::YarpCloudUtils::meshFromCloud(cloud, vertices, indices, options);
```

The above pipeline will first downsample the input cloud using the `pcl::VoxelGrid` algorithm
with a leaf size of 2 cm. Since `pcl::Poisson` requires an input normal type, we estimate
normals with an intermediate `NormalEstimation` step, and then perform surface reconstruction.

### Configuration context

Handy overloads are provided for both `meshFromCloud` and `processCloud` to parse the pipeline
from a configuration file or similar context (e.g. command line) using the YARP native
configuration format (see [YARP config files](https://www.yarp.it/git-master/yarp_config_files.html)).

The same behavior shown in the previous section can be achieved with an .ini file such as:

```ini
[myPipeline downsample]
algorithm "VoxelGrid"
leafSize 0.02f

[myPipeline estimate]
algorithm "NormalEstimationOMP"
kSearch 40

[myPipeline reconstruct]
algorithm "Poisson"
```

Then, in C++ code:

```cxx
yarp::os::Property config;
config.fromConfigFile("path/to/file.ini");

bool ret = roboticslab::YarpCloudUtils::meshFromCloud(cloud, vertices, indices, config, "myPipeline");
```

The `myPipeline` element is the name of the section collection which is common to all steps
in this configuration (see [YARP docs](https://www.yarp.it/git-master/yarp_config_files.html#yarp_config_file_lists)).
We can express this intent also directly in C++ code:

```cxx
yarp::os::Property config("(myPipeline downsample estimate reconstruct)");
config.addGroup("downsample") = {
    {"algorithm", yarp::os::Value("VoxelGrid")},
    {"leafSize", yarp::os::Value(0.02f)}
};
config.addGroup("estimate") = {
    {"algorithm", yarp::os::Value("NormalEstimationOMP")},
    {"kSearch", yarp::os::Value(40)}
};
config.addGroup("reconstruct") = {
    {"algorithm", yarp::os::Value("Poisson")}
};

bool ret = roboticslab::YarpCloudUtils::meshFromCloud(cloud, vertices, indices, config, "myPipeline");
```

Or, via command line:

```bash
application --myPipeline downsample estimate reconstruct \
            --downsample::algorithm VoxelGrid --downsample::leafSize 0.02f \
            --estimate::algorithm NormalEstimationOMP --estimate::kSearch 40 \
            --reconstruct::algorithm Poisson
```

```cxx
yarp::os::Property options;
options.fromCommand(argc, argv);
bool ret = roboticslab::YarpCloudUtils::meshFromCloud(cloud, vertices, indices, config, "myPipeline");
```

In case you want to maintain a collection of pipeline configurations, each one placed within
its own .ini file, then select the most appropriate for the task at hand, it is advised to
use the `include` feature combined with simultaneous section collection
([docs](https://www.yarp.it/git-master/yarp_config_files.html#yarp_config_file_includes)).
Refer to YARP documentation for further examples and a similar functionality related to
directory inclusion.
