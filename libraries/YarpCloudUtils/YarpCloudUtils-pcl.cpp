#include "YarpCloudUtils.hpp"

#include <yarp/pcl/Pcl.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

namespace
{
    template <typename T>
    struct pcl_type_from_yarp;

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXY>
    { typedef pcl::PointXY type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZ>
    { typedef pcl::PointXYZ type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataNormal>
    { typedef pcl::Normal type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZRGBA>
    { typedef pcl::PointXYZRGBA type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZI>
    { typedef pcl::PointXYZI type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataInterestPointXYZ>
    { typedef pcl::InterestPoint type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZNormal>
    { typedef pcl::PointNormal type; };

    template <>
    struct pcl_type_from_yarp<yarp::sig::DataXYZNormalRGBA>
    { typedef pcl::PointXYZRGBNormal type; };
}

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, bool isBinary)
{
    using pcl_type = typename pcl_type_from_yarp<T>::type;
    pcl::PointCloud<pcl_type> pcl;
    return yarp::pcl::toPCL(cloud, pcl) && pcl::io::savePLYFile(filename, pcl, isBinary) == 0;
}

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud)
{
    using pcl_type = typename pcl_type_from_yarp<T>::type;
    pcl::PointCloud<pcl_type> pcl;
    return pcl::io::loadPLYFile(filename, pcl) == 0 && yarp::pcl::fromPCL(pcl, cloud);
}

} // namespace YarpCloudUtils

} // namespace roboticslab

// explicit instantiations
#include "YarpCloudUtils-inst.hpp"
