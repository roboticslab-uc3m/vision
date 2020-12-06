// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CLOUD_UTILS_PCL_HPP__
#define __YARP_CLOUD_UTILS_PCL_HPP__

#include <stdexcept>
#include <type_traits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>

namespace
{

class cloud_container
{
public:
    template <typename T>
    typename pcl::PointCloud<T>::ConstPtr getCloud() const;

    template <typename T>
    typename pcl::PointCloud<T>::Ptr & setCloud();

    template <typename T>
    const typename pcl::PointCloud<T>::Ptr & useCloud() const;

    const pcl::PolygonMesh::Ptr & getMesh() const;

    pcl::PolygonMesh::Ptr & setMesh();

private:
    template <typename T1, typename T2, std::enable_if_t<!std::is_same<T1, T2>::value, bool> = true>
    static auto initializeCloudPointer(const typename pcl::PointCloud<T1>::ConstPtr & in);

    template <typename T1, typename T2, std::enable_if_t<std::is_same<T1, T2>::value, bool> = true>
    static auto initializeCloudPointer(const typename pcl::PointCloud<T1>::ConstPtr & in);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb;
    pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi;
    pcl::PointCloud<pcl::InterestPoint>::Ptr xyz_interest;
    pcl::PointCloud<pcl::PointNormal>::Ptr xyz_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr xyz_rgb_normal;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr xyzi_normal;
    pcl::PolygonMesh::Ptr mesh;
};

// helpers

template <typename T1, typename T2, std::enable_if_t<!std::is_same<T1, T2>::value, bool>>
inline auto cloud_container::initializeCloudPointer(const typename pcl::PointCloud<T1>::ConstPtr & in)
{
    typename pcl::PointCloud<T2>::Ptr out(new pcl::PointCloud<T2>());
    pcl::copyPointCloud(*in, *out);
    return out;
}

template <typename T1, typename T2, std::enable_if_t<std::is_same<T1, T2>::value, bool>>
inline auto cloud_container::initializeCloudPointer(const typename pcl::PointCloud<T1>::ConstPtr & in)
{
    return in;
}

// cloud_container::getCloud

template <typename T>
typename pcl::PointCloud<T>::ConstPtr cloud_container::getCloud() const
{
    if (xyz)
    {
        return initializeCloudPointer<pcl::PointXYZ, T>(xyz);
    }
    else if (xyz_rgb)
    {
        return initializeCloudPointer<pcl::PointXYZRGB, T>(xyz_rgb);
    }
    else if (xyzi)
    {
        return initializeCloudPointer<pcl::PointXYZI, T>(xyzi);
    }
    else if (xyz_interest)
    {
        return initializeCloudPointer<pcl::InterestPoint, T>(xyz_interest);
    }
    else if (xyz_normal)
    {
        return initializeCloudPointer<pcl::PointNormal, T>(xyz_normal);
    }
    else if (xyz_rgb_normal)
    {
        return initializeCloudPointer<pcl::PointXYZRGBNormal, T>(xyz_rgb_normal);
    }
    else if (xyzi_normal)
    {
        return initializeCloudPointer<pcl::PointXYZINormal, T>(xyzi_normal);
    }
    else
    {
        throw std::runtime_error("cloud pointer was not initialized");
    }
}

// cloud_container::setCloud

template <>
pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_container::setCloud<pcl::PointXYZ>()
{
    xyz.reset(new pcl::PointCloud<pcl::PointXYZ>());
    return xyz;
}

template <>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_container::setCloud<pcl::PointXYZRGB>()
{
    xyz_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    return xyz_rgb;
}

template <>
pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_container::setCloud<pcl::PointXYZI>()
{
    xyzi.reset(new pcl::PointCloud<pcl::PointXYZI>());
    return xyzi;
}

template <>
pcl::PointCloud<pcl::InterestPoint>::Ptr & cloud_container::setCloud<pcl::InterestPoint>()
{
    xyz_interest.reset(new pcl::PointCloud<pcl::InterestPoint>());
    return xyz_interest;
}

template <>
pcl::PointCloud<pcl::PointNormal>::Ptr & cloud_container::setCloud<pcl::PointNormal>()
{
    xyz_normal.reset(new pcl::PointCloud<pcl::PointNormal>());
    return xyz_normal;
}

template <>
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud_container::setCloud<pcl::PointXYZRGBNormal>()
{
    xyz_rgb_normal.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    return xyz_rgb_normal;
}

template <>
pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud_container::setCloud<pcl::PointXYZINormal>()
{
    xyzi_normal.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    return xyzi_normal;
}

// cloud_container::useCloud

template <>
const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_container::useCloud<pcl::PointXYZ>() const
{
    return xyz;
}

template <>
const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_container::useCloud<pcl::PointXYZRGB>() const
{
    return xyz_rgb;
}

template <>
const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_container::useCloud<pcl::PointXYZI>() const
{
    return xyzi;
}

template <>
const pcl::PointCloud<pcl::InterestPoint>::Ptr & cloud_container::useCloud<pcl::InterestPoint>() const
{
    return xyz_interest;
}

template <>
const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud_container::useCloud<pcl::PointNormal>() const
{
    return xyz_normal;
}

template <>
const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud_container::useCloud<pcl::PointXYZRGBNormal>() const
{
    return xyz_rgb_normal;
}

template <>
const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud_container::useCloud<pcl::PointXYZINormal>() const
{
    return xyzi_normal;
}

// cloud_container::(get|set)Mesh

inline const pcl::PolygonMesh::Ptr & cloud_container::getMesh() const
{
    if (mesh)
    {
        return mesh;
    }
    else
    {
        throw std::runtime_error("mesh pointer was not initialized");
    }
}

inline pcl::PolygonMesh::Ptr & cloud_container::setMesh()
{
    mesh.reset(new pcl::PolygonMesh());
    return mesh;
}

} // namespace

#endif // __YARP_CLOUD_UTILS_PCL_HPP__
