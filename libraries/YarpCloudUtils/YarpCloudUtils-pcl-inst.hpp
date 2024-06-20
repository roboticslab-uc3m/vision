// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CLOUD_UTILS_PCL_INST_HPP__
#define __YARP_CLOUD_UTILS_PCL_INST_HPP__

#include <string>

#include <yarp/os/Searchable.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/Vector.h>

#define YCU_PCL_SIGNATURES(in, out) \
    template bool meshFromCloud(const in &, out &, yarp::sig::VectorOf<int> &, const yarp::sig::VectorOf<yarp::os::Property> &); \
    template bool meshFromCloud(const in &, out &, yarp::sig::VectorOf<int> &, const yarp::os::Searchable &, const std::string &); \
    template bool processCloud(const in &, out &, const yarp::sig::VectorOf<yarp::os::Property> &); \
    template bool processCloud(const in &, out &, const yarp::os::Searchable &, const std::string &);

namespace roboticslab::YarpCloudUtils
{

YCU_PCL_SIGNATURES(yarp::sig::PointCloudXY, yarp::sig::PointCloudXY)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXY, yarp::sig::PointCloudXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXY, yarp::sig::PointCloudNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXY, yarp::sig::PointCloudXYZRGBA)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXY, yarp::sig::PointCloudXYZI)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXY, yarp::sig::PointCloudInterestPointXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXY, yarp::sig::PointCloudXYZNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXY, yarp::sig::PointCloudXYZNormalRGBA)

YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZ, yarp::sig::PointCloudXY)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZ, yarp::sig::PointCloudXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZ, yarp::sig::PointCloudNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZ, yarp::sig::PointCloudXYZRGBA)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZ, yarp::sig::PointCloudXYZI)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZ, yarp::sig::PointCloudInterestPointXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZ, yarp::sig::PointCloudXYZNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZ, yarp::sig::PointCloudXYZNormalRGBA)

YCU_PCL_SIGNATURES(yarp::sig::PointCloudNormal, yarp::sig::PointCloudXY)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudNormal, yarp::sig::PointCloudXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudNormal, yarp::sig::PointCloudNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudNormal, yarp::sig::PointCloudXYZRGBA)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudNormal, yarp::sig::PointCloudXYZI)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudNormal, yarp::sig::PointCloudInterestPointXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudNormal, yarp::sig::PointCloudXYZNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudNormal, yarp::sig::PointCloudXYZNormalRGBA)

YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZRGBA, yarp::sig::PointCloudXY)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZRGBA, yarp::sig::PointCloudXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZRGBA, yarp::sig::PointCloudNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZRGBA, yarp::sig::PointCloudXYZRGBA)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZRGBA, yarp::sig::PointCloudXYZI)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZRGBA, yarp::sig::PointCloudInterestPointXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZRGBA, yarp::sig::PointCloudXYZNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZRGBA, yarp::sig::PointCloudXYZNormalRGBA)

YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZI, yarp::sig::PointCloudXY)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZI, yarp::sig::PointCloudXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZI, yarp::sig::PointCloudNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZI, yarp::sig::PointCloudXYZRGBA)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZI, yarp::sig::PointCloudXYZI)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZI, yarp::sig::PointCloudInterestPointXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZI, yarp::sig::PointCloudXYZNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZI, yarp::sig::PointCloudXYZNormalRGBA)

YCU_PCL_SIGNATURES(yarp::sig::PointCloudInterestPointXYZ, yarp::sig::PointCloudXY)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudInterestPointXYZ, yarp::sig::PointCloudXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudInterestPointXYZ, yarp::sig::PointCloudNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudInterestPointXYZ, yarp::sig::PointCloudXYZRGBA)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudInterestPointXYZ, yarp::sig::PointCloudXYZI)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudInterestPointXYZ, yarp::sig::PointCloudInterestPointXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudInterestPointXYZ, yarp::sig::PointCloudXYZNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudInterestPointXYZ, yarp::sig::PointCloudXYZNormalRGBA)

YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormal, yarp::sig::PointCloudXY)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormal, yarp::sig::PointCloudXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormal, yarp::sig::PointCloudNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormal, yarp::sig::PointCloudXYZRGBA)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormal, yarp::sig::PointCloudXYZI)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormal, yarp::sig::PointCloudInterestPointXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormal, yarp::sig::PointCloudXYZNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormal, yarp::sig::PointCloudXYZNormalRGBA)

YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormalRGBA, yarp::sig::PointCloudXY)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormalRGBA, yarp::sig::PointCloudXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormalRGBA, yarp::sig::PointCloudNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormalRGBA, yarp::sig::PointCloudXYZRGBA)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormalRGBA, yarp::sig::PointCloudXYZI)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormalRGBA, yarp::sig::PointCloudInterestPointXYZ)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormalRGBA, yarp::sig::PointCloudXYZNormal)
YCU_PCL_SIGNATURES(yarp::sig::PointCloudXYZNormalRGBA, yarp::sig::PointCloudXYZNormalRGBA)

} // namespace roboticslab::YarpCloudUtils

#endif // __YARP_CLOUD_UTILS_PCL_INST_HPP__
