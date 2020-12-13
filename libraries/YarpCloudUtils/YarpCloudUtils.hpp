// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CLOUD_UTILS_HPP__
#define __YARP_CLOUD_UTILS_HPP__

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>

#include <yarp/sig/PointCloud.h>
#include <yarp/sig/Vector.h>

/**
 * @ingroup vision_libraries
 * @defgroup YarpCloudUtils
 * @brief Collection of cloud-related utilities for YARP.
 *
 * @see [Instructions](@ref yarpcloudutils).
 */

namespace roboticslab
{

/**
 * @ingroup YarpCloudUtils
 * @brief Collection of cloud-related utilities for YARP.
 */
namespace YarpCloudUtils
{

/**
 * @ingroup YarpCloudUtils
 * @brief Writes a triangular polygon mesh to file.
 *
 * @param filename Path to a file with .ply extension.
 * @param cloud Cloud of vertices.
 * @param indices Vector of indices, each three consecutive values define a face.
 * @param isBinary Whether to save file with binary format or not.
 *
 * @return Whether the mesh has been successfully exported or not.
 */
template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary = true);

/**
 * @ingroup YarpCloudUtils
 * @brief Writes a point cloud to file.
 *
 * @param filename Path to a file with .ply extension.
 * @param cloud Cloud of points.
 * @param isBinary Whether to save file with binary format or not.
 *
 * @return Whether the cloud has been successfully exported or not.
 */
template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, bool isBinary = true)
{
    return savePLY(filename, cloud, {}, isBinary);
}

/**
 * @ingroup YarpCloudUtils
 * @brief Reads a triangular polygon mesh from file.
 *
 * @note Failure is reported if required fields are missing, depending on the requested
 * point type. Optional fields are `alpha` (RGBA types) and `curvature` (normal types).
 *
 * @param filename Path to a file with .ply extension.
 * @param cloud Cloud of vertices.
 * @param indices Vector of indices, each three consecutive values define a face.
 *
 * @return Whether the mesh has been successfully imported or not.
 */
template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud, yarp::sig::VectorOf<int> & indices);

/**
 * @ingroup YarpCloudUtils
 * @brief Reads a point cloud from file.
 *
 * @note Failure is reported if required fields are missing, depending on the requested
 * point type. Optional fields are `alpha` (RGBA types) and `curvature` (normal types).
 *
 * @param filename Path to a file with .ply extension.
 * @param cloud Cloud of vertices.
 *
 * @return Whether the cloud has been successfully imported or not.
 */
template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud)
{
    yarp::sig::VectorOf<int> indices;
    return loadPLY(filename, cloud, indices);
}

/**
 * @ingroup YarpCloudUtils
 * @brief Constructs a triangular polygon mesh from a point cloud.
 *
 * @note Implements a set of PCL algorithms. Refer to [instructions](@ref yarpcloudutils).
 *
 * @param cloud Input cloud.
 * @param meshPoints Cloud of vertices of the resulting polygon mesh.
 * @param meshIndices Vector if indices of the resulting polygon mesh, each three
 * consecutive values define a face.
 * @param options Vector of dictionaries, each element defines a step of the pipeline.
 *
 * @return Whether any failure occurred throughout the pipeline.
 */
template <typename T1, typename T2 = T1>
bool meshFromCloud(const yarp::sig::PointCloud<T1> & cloud,
                   yarp::sig::PointCloud<T2> & meshPoints,
                   yarp::sig::VectorOf<int> & meshIndices,
                   const yarp::sig::VectorOf<yarp::os::Property> & options);

/**
 * @ingroup YarpCloudUtils
 * @brief Constructs a triangular polygon mesh from a point cloud.
 *
 * @note Implements a set of PCL algorithms. Refer to [instructions](@ref yarpcloudutils).
 *
 * @param cloud Input cloud.
 * @param meshPoints Cloud of vertices of the resulting polygon mesh.
 * @param meshIndices Vector if indices of the resulting polygon mesh, each three
 * consecutive values define a face.
 * @param config Configuration in YARP native format to read the pipeline from.
 * @param collection Named section collection that identifies this pipeline in @p config.
 *
 * @return Whether any failure occurred throughout the pipeline.
 */
template <typename T1, typename T2 = T1>
bool meshFromCloud(const yarp::sig::PointCloud<T1> & cloud,
                   yarp::sig::PointCloud<T2> & meshPoints,
                   yarp::sig::VectorOf<int> & meshIndices,
                   const yarp::os::Searchable & config,
                   const std::string & collection = "meshPipeline");

/**
 * @ingroup YarpCloudUtils
 * @brief Processes a cloud of points.
 *
 * @note Implements a set of PCL algorithms. Refer to [instructions](@ref yarpcloudutils).
 *
 * @param cloud Input cloud.
 * @param meshPoints Cloud of vertices of the resulting polygon mesh.
 * @param meshIndices Vector if indices of the resulting polygon mesh, each three
 * consecutive values define a face.
 * @param options Vector of dictionaries, each element defines a step of the pipeline.
 *
 * @return Whether any failure occurred throughout the pipeline.
 */
template <typename T1, typename T2 = T1>
bool processCloud(const yarp::sig::PointCloud<T1> & in,
                  yarp::sig::PointCloud<T2> & out,
                  const yarp::sig::VectorOf<yarp::os::Property> & options);

/**
 * @ingroup YarpCloudUtils
 * @brief Processes a cloud of points.
 *
 * @note Implements a set of PCL algorithms. Refer to [instructions](@ref yarpcloudutils).
 *
 * @param cloud Input cloud.
 * @param meshPoints Cloud of vertices of the resulting polygon mesh.
 * @param meshIndices Vector if indices of the resulting polygon mesh, each three
 * consecutive values define a face.
 * @param config Configuration in YARP native format to read the pipeline from.
 * @param collection Named section collection that identifies this pipeline in @p config.
 *
 * @return Whether any failure occurred throughout the pipeline.
 */
template <typename T1, typename T2 = T1>
bool processCloud(const yarp::sig::PointCloud<T1> & in,
                  yarp::sig::PointCloud<T2> & out,
                  const yarp::os::Searchable & config,
                  const std::string & collection = "cloudPipeline");

} // namespace YarpCloudUtils

} // namespace roboticslab

#endif // __YARP_CLOUD_UTILS_HPP__
