// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCloudUtils.hpp"

#include <cstring>
#include <memory>
#include <fstream>
#include <ostream>

#include "tinyply.h"

namespace
{
    void write(std::ostream & os, const yarp::sig::PointCloudXY & cloud, bool isBinary)
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

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZ & cloud, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzSize = sizeof(float) * 3;
        auto buffer = std::make_unique<unsigned char[]>(cloud.size() * xyzSize);

        for (auto i = 0; i < cloud.size(); i++)
        {
            std::memcpy(buffer.get() + i * xyzSize, cloud(i)._xyz, xyzSize);
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer.get(),
            tinyply::Type::INVALID,
            0);

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudNormal & cloud, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto normalSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer = std::make_unique<unsigned char[]>(cloud.size() * normalSize);

        for (auto i = 0; i < cloud.size(); i++)
        {
            std::memcpy(buffer.get() + i * normalSize, cloud(i).normal, offset);
            std::memcpy(buffer.get() + i * normalSize + offset, &cloud(i).curvature, sizeof(float));
        }

        ply.add_properties_to_element(
            "vertex",
            {"nx", "ny", "nz", "curvature"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer.get(),
            tinyply::Type::INVALID,
            0);

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZRGBA & cloud, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzSize = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * xyzSize);

        constexpr auto rgbaSize = 4;
        auto buffer_rgba = std::make_unique<unsigned char[]>(cloud.size() * rgbaSize);

        for (auto i = 0; i < cloud.size(); i++)
        {
            std::memcpy(buffer_xyz.get() + i * xyzSize, cloud(i)._xyz, xyzSize);
            std::memcpy(buffer_rgba.get() + i * rgbaSize, &cloud(i).rgba, rgbaSize);
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

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZI & cloud, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyziSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer = std::make_unique<unsigned char[]>(cloud.size() * xyziSize);

        for (auto i = 0; i < cloud.size(); i++)
        {
            std::memcpy(buffer.get() + i * xyziSize, cloud(i)._xyz, offset);
            std::memcpy(buffer.get() + i * xyziSize + offset, &cloud(i).intensity, sizeof(float));
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z", "intensity"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer.get(),
            tinyply::Type::INVALID,
            0);

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudInterestPointXYZ & cloud, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzintSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer = std::make_unique<unsigned char[]>(cloud.size() * xyzintSize);

        for (auto i = 0; i < cloud.size(); i++)
        {
            std::memcpy(buffer.get() + i * xyzintSize, cloud(i)._xyz, offset);
            std::memcpy(buffer.get() + i * xyzintSize + offset, &cloud(i).strength, sizeof(float));
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z", "strength"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer.get(),
            tinyply::Type::INVALID,
            0);

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZNormal & cloud, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzSize = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * xyzSize);

        constexpr auto normalSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_normal = std::make_unique<unsigned char[]>(cloud.size() * normalSize);

        for (auto i = 0; i < cloud.size(); i++)
        {
            std::memcpy(buffer_xyz.get() + i * xyzSize, cloud(i).data, xyzSize);
            std::memcpy(buffer_normal.get() + i * normalSize, cloud(i).normal, offset);
            std::memcpy(buffer_normal.get() + i * normalSize + offset, &cloud(i).curvature, sizeof(float));
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

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZNormalRGBA & cloud, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzSize = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * xyzSize);

        constexpr auto normalSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_normal = std::make_unique<unsigned char[]>(cloud.size() * normalSize);

        constexpr auto rgbaSize = 4;
        auto buffer_rgba = std::make_unique<unsigned char[]>(cloud.size() * rgbaSize);

        for (auto i = 0; i < cloud.size(); i++)
        {
            std::memcpy(buffer_xyz.get() + i * xyzSize, cloud(i).data, xyzSize);
            std::memcpy(buffer_normal.get() + i * normalSize, cloud(i).normal, offset);
            std::memcpy(buffer_normal.get() + i * normalSize + offset, &cloud(i).curvature, sizeof(float));
            std::memcpy(buffer_rgba.get() + i * rgbaSize, &cloud(i).rgba, rgbaSize);
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

        ply.write(os, isBinary);
    }
}

namespace roboticslab
{

namespace YarpCloudUtils
{

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, bool isBinary)
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
        return false;
    }

    write(os, cloud, isBinary);
    return true;
}

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud)
{
    return true;
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

template bool loadPLY(const std::string &, yarp::sig::PointCloudXY &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZ &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudNormal &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZRGBA &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZI &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudInterestPointXYZ &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZNormal &);
template bool loadPLY(const std::string &, yarp::sig::PointCloudXYZNormalRGBA &);

} // namespace YarpCloudUtils

} // namespace roboticslab
