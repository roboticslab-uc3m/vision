// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CLOUD_UTILS_HPP__
#define __YARP_CLOUD_UTILS_HPP__

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>

#include <yarp/sig/PointCloud.h>
#include <yarp/sig/Vector.h>

#include <SurfaceMeshingOptions.hpp>

/**
 * @ingroup vision_libraries
 * @defgroup YarpCloudUtils
 */

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary = true);

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, bool isBinary = true)
{
    return savePLY(filename, cloud, {}, isBinary);
}

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud, yarp::sig::VectorOf<int> & indices);

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud)
{
    yarp::sig::VectorOf<int> indices;
    return loadPLY(filename, cloud, indices);
}

template <typename T>
bool meshFromCloud(const yarp::sig::PointCloud<T> & cloud,
                   yarp::sig::PointCloud<T> & meshPoints,
                   yarp::sig::VectorOf<int> & meshIndices,
                   const yarp::os::Searchable & options = yarp::os::Property({
                                                              {"downsampleAlgorithm", yarp::os::Value("VoxelGrid")},
                                                              {"downsampleLeafSize", yarp::os::Value(0.02f)},
                                                              {"estimatorKSearch", yarp::os::Value(40)},
                                                              {"surfaceAlgorithm", yarp::os::Value("Poisson")}
                                                          }));

} // namespace YarpCloudUtils

} // namespace roboticslab

#endif // __YARP_CLOUD_UTILS_HPP__
