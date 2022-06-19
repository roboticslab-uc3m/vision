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

struct pcl_xyz_rgb_types_tag {}; // XYZ or XYZ+RGB

struct pcl_rgb_types_tag {}; // XYZ+RGB

struct pcl_xyzi_types_tag {}; // XYZI(+Normal)

struct pcl_normal_types_tag {};

// map PCL type according to selected tag

template <typename T, typename tag>
struct pcl_decay;

// 1-to-1 mapping if T belongs to any of the supported XYZ types

template <typename T>
struct pcl_decay<T, pcl_all_xyz_types_tag>
{ typedef T type; };

// mappings for XYZ or XYZ+RGB types

template <typename T>
struct pcl_decay<T, pcl_xyz_rgb_types_tag>
{ typedef pcl::PointXYZ type; };

template <>
struct pcl_decay<pcl::PointXYZRGB, pcl_xyz_rgb_types_tag>
{ typedef pcl::PointXYZRGB type; };

template <>
struct pcl_decay<pcl::PointXYZRGBNormal, pcl_xyz_rgb_types_tag>
{ typedef pcl::PointXYZRGB type; };

// mappings for XYZ+RGB types

template <typename T>
struct pcl_decay<T, pcl_rgb_types_tag>
{ typedef pcl::PointXYZRGB type; };

// mappings for XYZI(+Normal) types

template <typename T>
struct pcl_decay<T, pcl_xyzi_types_tag>
{ typedef pcl::PointXYZI type; };

template <>
struct pcl_decay<pcl::PointNormal, pcl_xyzi_types_tag>
{ typedef pcl::PointXYZINormal type; };

template <>
struct pcl_decay<pcl::PointXYZRGBNormal, pcl_xyzi_types_tag>
{ typedef pcl::PointXYZINormal type; };

template <>
struct pcl_decay<pcl::PointXYZINormal, pcl_xyzi_types_tag>
{ typedef pcl::PointXYZINormal type; };

// mappings for XYZ+normal types

template <typename T>
struct pcl_decay<T, pcl_normal_types_tag>
{ typedef T type; };

template <>
struct pcl_decay<pcl::PointXYZ, pcl_normal_types_tag>
{ typedef pcl::PointNormal type; };

template <>
struct pcl_decay<pcl::PointXYZRGB, pcl_normal_types_tag>
{ typedef pcl::PointXYZRGBNormal type; };

template <>
struct pcl_decay<pcl::PointXYZI, pcl_normal_types_tag>
{ typedef pcl::PointXYZINormal type; };

template <>
struct pcl_decay<pcl::InterestPoint, pcl_normal_types_tag>
{ typedef pcl::PointNormal type; };

// register allowed conversions

template <typename T1, typename T2>
struct pcl_is_convertible : std::false_type {};

template <typename T>
struct pcl_is_convertible<T, pcl::PointXYZ> : std::true_type {}; // T->XYZ always allowed

template <>
struct pcl_is_convertible<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> : std::true_type {};

template <>
struct pcl_is_convertible<pcl::PointXYZRGBNormal, pcl::PointNormal> : std::true_type {};

template <>
struct pcl_is_convertible<pcl::PointXYZINormal, pcl::PointXYZI> : std::true_type {};

template <>
struct pcl_is_convertible<pcl::PointXYZINormal, pcl::PointNormal> : std::true_type {};

// describe each type

template <typename T>
struct pcl_descriptor;

template <>
struct pcl_descriptor<pcl::PointXY>
{ static constexpr const char * name = "XY"; };

template <>
struct pcl_descriptor<pcl::PointXYZ>
{ static constexpr const char * name = "XYZ"; };

template <>
struct pcl_descriptor<pcl::Normal>
{ static constexpr const char * name = "NORMAL"; };

template <>
struct pcl_descriptor<pcl::PointXYZRGB>
{ static constexpr const char * name = "XYZ_RGB"; };

template <>
struct pcl_descriptor<pcl::PointXYZI>
{ static constexpr const char * name = "XYZI"; };

template <>
struct pcl_descriptor<pcl::InterestPoint>
{ static constexpr const char * name = "XYZ_INTEREST"; };

template <>
struct pcl_descriptor<pcl::PointNormal>
{ static constexpr const char * name = "XYZ_NORMAL"; };

template <>
struct pcl_descriptor<pcl::PointXYZRGBNormal>
{ static constexpr const char * name = "XYZ_RGB_NORMAL"; };

template <>
struct pcl_descriptor<pcl::PointXYZINormal>
{ static constexpr const char * name = "XYZI_NORMAL"; };

// conditional switches

template <typename T>
constexpr auto is_unsupported_type =
    std::is_same_v<T, pcl::PointXY> ||
    std::is_same_v<T, pcl::Normal>;

} // namespace

#endif // __YARP_CLOUD_UTILS_PCL_TRAITS_HPP__
