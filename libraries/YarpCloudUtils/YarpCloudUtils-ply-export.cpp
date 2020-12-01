// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCloudUtils.hpp"

#include <cstring> // std::memcpy
#include <exception>
#include <fstream> // std::filebuf
#include <memory> // std::make_unique
#include <ostream>

#include <yarp/os/LogStream.h>

#include "tinyply.h"

namespace
{
    void write(std::ostream & os, const yarp::sig::PointCloudXY & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
    {
        tinyply::PlyFile ply;

        ply.add_properties_to_element(
            "vertex",
            {"x", "y"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            reinterpret_cast<unsigned char *>(const_cast<char *>(cloud.getRawData())),
            tinyply::Type::INVALID,
            0);

        if (indices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                indices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(indices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZ & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto size_xyz = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * size_xyz);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyz.get() + n * size_xyz, cloud(n)._xyz, size_xyz);
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_xyz.get(),
            tinyply::Type::INVALID,
            0);

        if (indices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                indices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(indices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudNormal & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto size_normal = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_normal = std::make_unique<unsigned char[]>(cloud.size() * size_normal);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_normal.get() + n * size_normal, cloud(n).normal, offset);
            std::memcpy(buffer_normal.get() + n * size_normal + offset, &cloud(n).curvature, size_normal - offset);
        }

        ply.add_properties_to_element(
            "vertex",
            {"nx", "ny", "nz", "curvature"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_normal.get(),
            tinyply::Type::INVALID,
            0);

        if (indices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                indices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(indices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZRGBA & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto size_xyz = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * size_xyz);

        constexpr auto size_rgba = sizeof(unsigned char) * 4;
        auto buffer_rgba = std::make_unique<unsigned char[]>(cloud.size() * size_rgba);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyz.get() + n * size_xyz, cloud(n)._xyz, size_xyz);
            std::memcpy(buffer_rgba.get() + n * size_rgba, &cloud(n).rgba, size_rgba);
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_xyz.get(),
            tinyply::Type::INVALID,
            0);

        ply.add_properties_to_element(
            "vertex",
            {"blue", "green", "red", "alpha"},
            tinyply::Type::UINT8,
            cloud.size(),
            buffer_rgba.get(),
            tinyply::Type::INVALID,
            0);

        if (indices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                indices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(indices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZI & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto size_xyzi = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_xyzi = std::make_unique<unsigned char[]>(cloud.size() * size_xyzi);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyzi.get() + n * size_xyzi, cloud(n)._xyz, offset);
            std::memcpy(buffer_xyzi.get() + n * size_xyzi + offset, &cloud(n).intensity, size_xyzi - offset);
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z", "intensity"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_xyzi.get(),
            tinyply::Type::INVALID,
            0);

        if (indices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                indices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(indices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudInterestPointXYZ & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto size_xyzint = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_xyzint = std::make_unique<unsigned char[]>(cloud.size() * size_xyzint);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyzint.get() + n * size_xyzint, cloud(n)._xyz, offset);
            std::memcpy(buffer_xyzint.get() + n * size_xyzint + offset, &cloud(n).strength, size_xyzint - offset);
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z", "strength"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_xyzint.get(),
            tinyply::Type::INVALID,
            0);

        if (indices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                indices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(indices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZNormal & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto size_xyz = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * size_xyz);

        constexpr auto size_normal = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_normal = std::make_unique<unsigned char[]>(cloud.size() * size_normal);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyz.get() + n * size_xyz, cloud(n).data, size_xyz);
            std::memcpy(buffer_normal.get() + n * size_normal, cloud(n).normal, offset);
            std::memcpy(buffer_normal.get() + n * size_normal + offset, &cloud(n).curvature, size_normal - offset);
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_xyz.get(),
            tinyply::Type::INVALID,
            0);

        ply.add_properties_to_element(
            "vertex",
            {"nx", "ny", "nz", "curvature"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_normal.get(),
            tinyply::Type::INVALID,
            0);

        if (indices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                indices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(indices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZNormalRGBA & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto size_xyz = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * size_xyz);

        constexpr auto size_normal = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_normal = std::make_unique<unsigned char[]>(cloud.size() * size_normal);

        constexpr auto size_rgba = sizeof(unsigned char) * 4;
        auto buffer_rgba = std::make_unique<unsigned char[]>(cloud.size() * size_rgba);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyz.get() + n * size_xyz, cloud(n).data, size_xyz);
            std::memcpy(buffer_normal.get() + n * size_normal, cloud(n).normal, offset);
            std::memcpy(buffer_normal.get() + n * size_normal + offset, &cloud(n).curvature, size_normal - offset);
            std::memcpy(buffer_rgba.get() + n * size_rgba, &cloud(n).rgba, size_rgba);
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_xyz.get(),
            tinyply::Type::INVALID,
            0);

        ply.add_properties_to_element(
            "vertex",
            {"nx", "ny", "nz", "curvature"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer_normal.get(),
            tinyply::Type::INVALID,
            0);

        ply.add_properties_to_element(
            "vertex",
            {"blue", "green", "red", "alpha"},
            tinyply::Type::UINT8,
            cloud.size(),
            buffer_rgba.get(),
            tinyply::Type::INVALID,
            0);

        if (indices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                indices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(indices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }
}

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, const yarp::sig::VectorOf<int> & indices, bool isBinary)
{
    auto modes = std::ios::out;

    if (isBinary)
    {
        modes |= std::ios::binary;
    }

    std::filebuf fb;
    fb.open(filename, modes);

    std::ostream os(&fb);

    if (os.fail())
    {
        yError() << "unable to open" << filename << "for write";
        return false;
    }

    try
    {
        write(os, cloud, indices, isBinary);
        return true;
    }
    catch (const std::exception & e)
    {
        yError() << e.what();
        return false;
    }
}

// explicit instantiations

template bool savePLY(const std::string &, const yarp::sig::PointCloudXY &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZ &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudNormal &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZRGBA &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZI &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudInterestPointXYZ &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZNormal &, bool);
template bool savePLY(const std::string &, const yarp::sig::PointCloudXYZNormalRGBA &, bool);

} // namespace YarpCloudUtils

} // namespace roboticslab
