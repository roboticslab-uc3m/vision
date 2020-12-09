// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CLOUD_UTILS_PCL_TRAITS_HPP__
#define __YARP_CLOUD_UTILS_PCL_TRAITS_HPP__

#include <type_traits>
#include <yarp/sig/PointCloudTypes.h>
#include <pcl/point_types.h>

namespace
{

// YARP point type to PCL point type

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
{ typedef pcl::PointXYZRGB type; };

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

// PCL type tags

struct pcl_all_xyz_types_tag {};

struct pcl_xyz_rgb_types_tag {};

struct pcl_normal_types_tag {};

// Convert PCL type according to selected tag

template <typename T, typename tag>
struct pcl_convert;

// 1-to-1 mapping if T belongs to any of the supported XYZ types

template <typename T>
struct pcl_convert<T, pcl_all_xyz_types_tag>
{ typedef T type; };

// Mappings for XYZ(RGB) types

template <typename T>
struct pcl_convert<T, pcl_xyz_rgb_types_tag>
{ typedef T type; };

template <>
struct pcl_convert<pcl::PointXYZI, pcl_xyz_rgb_types_tag>
{ typedef pcl::PointXYZ type; };

template <>
struct pcl_convert<pcl::InterestPoint, pcl_xyz_rgb_types_tag>
{ typedef pcl::PointXYZ type; };

template <>
struct pcl_convert<pcl::PointNormal, pcl_xyz_rgb_types_tag>
{ typedef pcl::PointXYZ type; };

template <>
struct pcl_convert<pcl::PointXYZRGBNormal, pcl_xyz_rgb_types_tag>
{ typedef pcl::PointXYZRGB type; };

// Mappings for XYZ+normal types

template <typename T>
struct pcl_convert<T, pcl_normal_types_tag>
{ typedef T type; };

template <>
struct pcl_convert<pcl::PointXYZ, pcl_normal_types_tag>
{ typedef pcl::PointNormal type; };

template <>
struct pcl_convert<pcl::PointXYZRGB, pcl_normal_types_tag>
{ typedef pcl::PointXYZRGBNormal type; };

template <>
struct pcl_convert<pcl::PointXYZI, pcl_normal_types_tag>
{ typedef pcl::PointXYZINormal type; };

template <>
struct pcl_convert<pcl::InterestPoint, pcl_normal_types_tag>
{ typedef pcl::PointNormal type; };

// Describe each type

template <typename T>
struct descriptor;

template <>
struct descriptor<pcl::PointXY>
{ static constexpr const char * name = "XY"; };

template <>
struct descriptor<pcl::PointXYZ>
{ static constexpr const char * name = "XYZ"; };

template <>
struct descriptor<pcl::Normal>
{ static constexpr const char * name = "NORMAL"; };

template <>
struct descriptor<pcl::PointXYZRGB>
{ static constexpr const char * name = "XYZ_RGB"; };

template <>
struct descriptor<pcl::PointXYZI>
{ static constexpr const char * name = "XYZI"; };

template <>
struct descriptor<pcl::InterestPoint>
{ static constexpr const char * name = "XYZ_INTEREST"; };

template <>
struct descriptor<pcl::PointNormal>
{ static constexpr const char * name = "XYZ_NORMAL"; };

template <>
struct descriptor<pcl::PointXYZRGBNormal>
{ static constexpr const char * name = "XYZ_RGB_NORMAL"; };

// Conditional switches

template <typename T>
constexpr auto is_unsupported_type =
    std::is_same<T, pcl::PointXY>::value ||
    std::is_same<T, pcl::Normal>::value;

} // namespace

#endif // __YARP_CLOUD_UTILS_PCL_TRAITS_HPP__
