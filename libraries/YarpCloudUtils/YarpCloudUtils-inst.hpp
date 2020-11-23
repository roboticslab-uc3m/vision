#ifndef __YARP_CLOUD_UTILS_INST_HPP__
#define __YARP_CLOUD_UTILS_INST_HPP__

#include "YarpCloudUtils.hpp"

namespace roboticslab
{
namespace YarpCloudUtils
{

template bool savePLY(const std::string &, const yarp::sig::PointCloudXY &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZ &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudNormal &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZRGBA &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZI &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudInterestPointXYZ &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZNormal &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZNormalRGBA &, bool);

template bool savePLY(const std::string &, const yarp::sig::PointCloudXY &, const yarp::sig::VectorOf<int> &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZ &, const yarp::sig::VectorOf<int> &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudNormal &, const yarp::sig::VectorOf<int> &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZRGBA &, const yarp::sig::VectorOf<int> &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZI &, const yarp::sig::VectorOf<int> &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudInterestPointXYZ &, const yarp::sig::VectorOf<int> &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZNormal &, const yarp::sig::VectorOf<int> &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZNormalRGBA &, const yarp::sig::VectorOf<int> &, bool);

template bool loadPLY(const std::string &, yarp::sig::PointCloudXY &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZ &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudNormal &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZRGBA &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZI &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudInterestPointXYZ &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZNormal &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZNormalRGBA &);

template bool loadPLY(const std::string &, yarp::sig::PointCloudXY &, yarp::sig::VectorOf<int> &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZ &, yarp::sig::VectorOf<int> &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudNormal &, yarp::sig::VectorOf<int> &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZRGBA &, yarp::sig::VectorOf<int> &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZI &, yarp::sig::VectorOf<int> &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudInterestPointXYZ &, yarp::sig::VectorOf<int> &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZNormal &, yarp::sig::VectorOf<int> &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZNormalRGBA &, yarp::sig::VectorOf<int> &);

} // namespace YarpCloudUtils
} // namespace roboticslab

#endif // __YARP_CLOUD_UTILS_INST_HPP__
