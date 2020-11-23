// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CLOUD_UTILS_HPP__
#define __YARP_CLOUD_UTILS_HPP__

#include <string>

#include <yarp/sig/PointCloud.h>
#include <yarp/sig/Vector.h>

/**
 * @ingroup vision_libraries
 * @defgroup YarpCloudUtils
 */

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, bool isBinary);

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary);

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud);

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud, yarp::sig::VectorOf<int> & vertices);

} // namespace YarpCloudUtils

} // namespace roboticslab

#endif // __YARP_CLOUD_UTILS_HPP__
