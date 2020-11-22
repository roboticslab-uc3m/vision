// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCloudUtils.hpp"

#include <cstring>
#include <exception>
#include <fstream>
#include <istream>
#include <memory>
#include <ostream>
#include <vector>

#include <yarp/os/LogStream.h>

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

        constexpr auto rgbaSize = sizeof(unsigned char) * 4;
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

        constexpr auto rgbaSize = sizeof(unsigned char) * 4;
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

    bool read(std::ifstream & ifs, yarp::sig::PointCloudXY & cloud)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            auto vertices = file.request_properties_from_element("vertex", {"x", "y"});
            file.read(ifs);
            cloud.fromExternalPC(reinterpret_cast<const char *>(vertices->buffer.get_const()), cloud.getPointType(), vertices->count, 1);
            return true;
        }

        return false;
    }

    bool read(std::ifstream & ifs, yarp::sig::PointCloudXYZ & cloud)
    {
        tinyply::PlyFile file;
        
        if (file.parse_header(ifs))
        {
            auto vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
            file.read(ifs);

            constexpr auto xyzSize = sizeof(float) * 3;
            auto buffer = std::make_unique<unsigned char[]>(vertices->count * xyzSize);
            std::memcpy(buffer.get(), vertices->buffer.get_const(), vertices->buffer.size_bytes());
            cloud.resize(vertices->count);
            
            for (auto i = 0; i < vertices->count; i++)
            {
                std::memcpy(cloud(i)._xyz, buffer.get() + i * xyzSize, xyzSize);
            }

            return true;
        }

        return false;
    }

    bool read(std::ifstream & ifs, yarp::sig::PointCloudNormal & cloud)
    {
        tinyply::PlyFile file;
        
        if (file.parse_header(ifs))
        {
            auto normals = file.request_properties_from_element("vertex", {"nx", "ny", "nz", "curvature"});
            file.read(ifs);

            constexpr auto normalSize = sizeof(float) * 4;
            constexpr auto offset = sizeof(float) * 3;
            auto buffer_normal = std::make_unique<unsigned char[]>(normals->count * normalSize);
            std::memcpy(buffer_normal.get(), normals->buffer.get_const(), normals->buffer.size_bytes());
            cloud.resize(normals->count);
            
            for (auto i = 0; i < normals->count; i++)
            {
                std::memcpy(cloud(i).normal, buffer_normal.get() + i * normalSize, offset);
                std::memcpy(&cloud(i).curvature, buffer_normal.get() + i * normalSize + offset, sizeof(float));
            }

            return true;
        }

        return false;
    }

    bool read(std::ifstream & ifs, yarp::sig::PointCloudXYZRGBA & cloud)
    {
        tinyply::PlyFile file;
        
        if (file.parse_header(ifs))
        {
            auto vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
            auto rgba = file.request_properties_from_element("vertex", {"blue", "green", "red", "alpha"});
            file.read(ifs);

            constexpr auto xyzSize = sizeof(float) * 3;
            auto buffer_xyz = std::make_unique<unsigned char[]>(vertices->count * xyzSize);
            std::memcpy(buffer_xyz.get(), vertices->buffer.get_const(), vertices->buffer.size_bytes());

            constexpr auto rgbaSize = sizeof(unsigned char) * 4;
            auto buffer_rgba = std::make_unique<unsigned char[]>(rgba->count * rgbaSize);
            std::memcpy(buffer_rgba.get(), rgba->buffer.get_const(), rgba->buffer.size_bytes());

            cloud.resize(vertices->count);
            
            for (auto i = 0; i < vertices->count; i++)
            {
                std::memcpy(cloud(i)._xyz, buffer_xyz.get() + i * xyzSize, xyzSize);
                std::memcpy(&cloud(i).rgba, buffer_rgba.get() + i * rgbaSize, rgbaSize);
            }

            return true;
        }

        return false;
    }

    bool read(std::ifstream & ifs, yarp::sig::PointCloudXYZI & cloud)
    {
        tinyply::PlyFile file;
        
        if (file.parse_header(ifs))
        {
            auto xyzi = file.request_properties_from_element("vertex", {"x", "y", "z", "intensity"});
            file.read(ifs);

            constexpr auto xyziSize = sizeof(float) * 4;
            constexpr auto offset = sizeof(float) * 3;
            auto buffer_xyzi = std::make_unique<unsigned char[]>(xyzi->count * xyziSize);
            std::memcpy(buffer_xyzi.get(), xyzi->buffer.get_const(), xyzi->buffer.size_bytes());
            cloud.resize(xyzi->count);
            
            for (auto i = 0; i < xyzi->count; i++)
            {
                std::memcpy(cloud(i)._xyz, buffer_xyzi.get() + i * xyziSize, offset);
                std::memcpy(&cloud(i).intensity, buffer_xyzi.get() + i * xyziSize + offset, sizeof(float));
            }

            return true;
        }

        return false;
    }

    bool read(std::ifstream & ifs, yarp::sig::PointCloudInterestPointXYZ & cloud)
    {
        tinyply::PlyFile file;
        
        if (file.parse_header(ifs))
        {
            auto xyzint = file.request_properties_from_element("vertex", {"x", "y", "z", "strength"});
            file.read(ifs);

            constexpr auto xyzintSize = sizeof(float) * 4;
            constexpr auto offset = sizeof(float) * 3;
            auto buffer_xyzint = std::make_unique<unsigned char[]>(xyzint->count * xyzintSize);
            std::memcpy(buffer_xyzint.get(), xyzint->buffer.get_const(), xyzint->buffer.size_bytes());
            cloud.resize(xyzint->count);
            
            for (auto i = 0; i < xyzint->count; i++)
            {
                std::memcpy(cloud(i)._xyz, buffer_xyzint.get() + i * xyzintSize, offset);
                std::memcpy(&cloud(i).strength, buffer_xyzint.get() + i * xyzintSize + offset, sizeof(float));
            }

            return true;
        }

        return false;
    }

    bool read(std::ifstream & ifs, yarp::sig::PointCloudXYZNormal & cloud)
    {
        tinyply::PlyFile file;
        
        if (file.parse_header(ifs))
        {
            auto vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
            auto normals = file.request_properties_from_element("vertex", {"nx", "ny", "nz", "curvature"});
            file.read(ifs);

            constexpr auto xyzSize = sizeof(float) * 3;
            auto buffer_xyz = std::make_unique<unsigned char[]>(vertices->count * xyzSize);
            std::memcpy(buffer_xyz.get(), vertices->buffer.get_const(), vertices->buffer.size_bytes());

            constexpr auto normalSize = sizeof(float) * 4;
            constexpr auto offset = sizeof(float) * 3;
            auto buffer_normal = std::make_unique<unsigned char[]>(normals->count * normalSize);
            std::memcpy(buffer_normal.get(), normals->buffer.get_const(), normals->buffer.size_bytes());

            cloud.resize(vertices->count);
            
            for (auto i = 0; i < vertices->count; i++)
            {
                std::memcpy(cloud(i).data, buffer_xyz.get() + i * xyzSize, xyzSize);
                std::memcpy(cloud(i).normal, buffer_normal.get() + i * normalSize, offset);
                std::memcpy(&cloud(i).curvature, buffer_normal.get() + i * normalSize + offset, sizeof(float));
            }

            return true;
        }

        return false;
    }

    bool read(std::ifstream & ifs, yarp::sig::PointCloudXYZNormalRGBA & cloud)
    {
        tinyply::PlyFile file;
        
        if (file.parse_header(ifs))
        {
            auto vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
            auto normals = file.request_properties_from_element("vertex", {"nx", "ny", "nz", "curvature"});
            auto rgba = file.request_properties_from_element("vertex", {"blue", "green", "red", "alpha"});
            file.read(ifs);

            constexpr auto xyzSize = sizeof(float) * 3;
            auto buffer_xyz = std::make_unique<unsigned char[]>(vertices->count * xyzSize);
            std::memcpy(buffer_xyz.get(), vertices->buffer.get_const(), vertices->buffer.size_bytes());

            constexpr auto normalSize = sizeof(float) * 4;
            constexpr auto offset = sizeof(float) * 3;
            auto buffer_normal = std::make_unique<unsigned char[]>(normals->count * normalSize);
            std::memcpy(buffer_normal.get(), normals->buffer.get_const(), normals->buffer.size_bytes());

            constexpr auto rgbaSize = sizeof(unsigned char) * 4;
            auto buffer_rgba = std::make_unique<unsigned char[]>(rgba->count * rgbaSize);
            std::memcpy(buffer_rgba.get(), rgba->buffer.get_const(), rgba->buffer.size_bytes());

            cloud.resize(vertices->count);
            
            for (auto i = 0; i < vertices->count; i++)
            {
                std::memcpy(cloud(i).data, buffer_xyz.get() + i * xyzSize, xyzSize);
                std::memcpy(cloud(i).normal, buffer_normal.get() + i * normalSize, offset);
                std::memcpy(&cloud(i).curvature, buffer_normal.get() + i * normalSize + offset, sizeof(float));
                std::memcpy(&cloud(i).rgba, buffer_rgba.get() + i * rgbaSize, rgbaSize);
            }

            return true;
        }

        return false;
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
        yError() << "unable to open" << filename << "for write";
        return false;
    }

    try
    {
        write(os, cloud, isBinary);
        return true;
    }
    catch (const std::exception & e)
    {
        yError() << e.what();
        return false;
    }
}

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud)
{
    std::ifstream ifs(filename);

    if (ifs.fail())
    {
        yError() << "unable to open" << filename << "for read";
        return false;
    }

    try
    {
        return read(ifs, cloud);
    }
    catch (const std::exception & e)
    {
        yError() << e.what();
        return false;
    }
}

} // namespace YarpCloudUtils

} // namespace roboticslab

// explicit instantiations
#include "YarpCloudUtils-inst.hpp"
