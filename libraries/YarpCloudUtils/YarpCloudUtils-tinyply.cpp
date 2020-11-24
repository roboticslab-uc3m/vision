// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCloudUtils.hpp"

#include <cstring>
#include <algorithm>
#include <exception>
#include <fstream>
#include <istream>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <streambuf>
#include <vector>

#include <yarp/os/LogStream.h>

#include "tinyply.h"

namespace
{
    // https://github.com/ddiakopoulos/tinyply/blob/master/source/example-utils.hpp

    struct memory_buffer : public std::streambuf
    {
        char * p_start {nullptr};
        char * p_end {nullptr};
        std::size_t size;

        memory_buffer(char const * first_elem, std::size_t size)
            : p_start(const_cast<char *>(first_elem)), p_end(p_start + size), size(size)
        {
            setg(p_start, p_start, p_end);
        }

        pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override
        {
            if (dir == std::ios_base::cur) gbump(static_cast<int>(off));
            else setg(p_start, (dir == std::ios_base::beg ? p_start : p_end) + off, p_end);
            return gptr() - p_start;
        }

        pos_type seekpos(pos_type pos, std::ios_base::openmode which) override
        {
            return seekoff(pos, std::ios_base::beg, which);
        }
    };

    struct memory_stream : virtual memory_buffer, public std::istream
    {
        memory_stream(char const * first_elem, size_t size)
            : memory_buffer(first_elem, size), std::istream(static_cast<std::streambuf *>(this)) {}
    };

    void write(std::ostream & os, const yarp::sig::PointCloudXY & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
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

        if (vertices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                vertices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(vertices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZ & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzSize = sizeof(float) * 3;
        auto buffer = std::make_unique<unsigned char[]>(cloud.size() * xyzSize);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer.get() + n * xyzSize, cloud(n)._xyz, xyzSize);
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer.get(),
            tinyply::Type::INVALID,
            0);

        if (vertices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                vertices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(vertices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudNormal & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto normalSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer = std::make_unique<unsigned char[]>(cloud.size() * normalSize);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer.get() + n * normalSize, cloud(n).normal, offset);
            std::memcpy(buffer.get() + n * normalSize + offset, &cloud(n).curvature, sizeof(float));
        }

        ply.add_properties_to_element(
            "vertex",
            {"nx", "ny", "nz", "curvature"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer.get(),
            tinyply::Type::INVALID,
            0);

        if (vertices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                vertices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(vertices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZRGBA & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzSize = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * xyzSize);

        constexpr auto rgbaSize = sizeof(unsigned char) * 4;
        auto buffer_rgba = std::make_unique<unsigned char[]>(cloud.size() * rgbaSize);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyz.get() + n * xyzSize, cloud(n)._xyz, xyzSize);
            std::memcpy(buffer_rgba.get() + n * rgbaSize, &cloud(n).rgba, rgbaSize);
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

        if (vertices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                vertices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(vertices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZI & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyziSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer = std::make_unique<unsigned char[]>(cloud.size() * xyziSize);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer.get() + n * xyziSize, cloud(n)._xyz, offset);
            std::memcpy(buffer.get() + n * xyziSize + offset, &cloud(n).intensity, sizeof(float));
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z", "intensity"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer.get(),
            tinyply::Type::INVALID,
            0);

        if (vertices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                vertices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(vertices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudInterestPointXYZ & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzintSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer = std::make_unique<unsigned char[]>(cloud.size() * xyzintSize);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer.get() + n * xyzintSize, cloud(n)._xyz, offset);
            std::memcpy(buffer.get() + n * xyzintSize + offset, &cloud(n).strength, sizeof(float));
        }

        ply.add_properties_to_element(
            "vertex",
            {"x", "y", "z", "strength"},
            tinyply::Type::FLOAT32,
            cloud.size(),
            buffer.get(),
            tinyply::Type::INVALID,
            0);

        if (vertices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                vertices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(vertices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZNormal & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzSize = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * xyzSize);

        constexpr auto normalSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_normal = std::make_unique<unsigned char[]>(cloud.size() * normalSize);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyz.get() + n * xyzSize, cloud(n).data, xyzSize);
            std::memcpy(buffer_normal.get() + n * normalSize, cloud(n).normal, offset);
            std::memcpy(buffer_normal.get() + n * normalSize + offset, &cloud(n).curvature, sizeof(float));
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

        if (vertices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                vertices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(vertices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    void write(std::ostream & os, const yarp::sig::PointCloudXYZNormalRGBA & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
    {
        tinyply::PlyFile ply;

        constexpr auto xyzSize = sizeof(float) * 3;
        auto buffer_xyz = std::make_unique<unsigned char[]>(cloud.size() * xyzSize);

        constexpr auto normalSize = sizeof(float) * 4;
        constexpr auto offset = sizeof(float) * 3;
        auto buffer_normal = std::make_unique<unsigned char[]>(cloud.size() * normalSize);

        constexpr auto rgbaSize = sizeof(unsigned char) * 4;
        auto buffer_rgba = std::make_unique<unsigned char[]>(cloud.size() * rgbaSize);

        for (auto n = 0; n < cloud.size(); n++)
        {
            std::memcpy(buffer_xyz.get() + n * xyzSize, cloud(n).data, xyzSize);
            std::memcpy(buffer_normal.get() + n * normalSize, cloud(n).normal, offset);
            std::memcpy(buffer_normal.get() + n * normalSize + offset, &cloud(n).curvature, sizeof(float));
            std::memcpy(buffer_rgba.get() + n * rgbaSize, &cloud(n).rgba, rgbaSize);
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

        if (vertices.size() != 0)
        {
            ply.add_properties_to_element(
                "face",
                {"vertex_indices"},
                tinyply::Type::INT32,
                vertices.size() / 3,
                reinterpret_cast<unsigned char *>(const_cast<char *>(vertices.getMemoryBlock())),
                tinyply::Type::UINT8,
                3);
        }

        ply.write(os, isBinary);
    }

    auto getNumberOfElements(const tinyply::PlyFile & file, const std::string & name)
    {
        auto els = file.get_elements();
        const auto it = std::find_if(els.cbegin(), els.cend(), [&name](const auto & el) { return el.name == name; });

        if (it == els.cend())
        {
            throw std::invalid_argument("unknown element: " + name);
        }

        return it->size;
    }

    bool read(std::istream & ifs, yarp::sig::PointCloudXY & cloud, yarp::sig::VectorOf<int> & vertices)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            std::shared_ptr<tinyply::PlyData> faces;

            auto x = file.request_properties_from_element("vertex", {"x"});
            auto y = file.request_properties_from_element("vertex", {"y"});

            try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
            catch (...) {}

            file.read(ifs);
            cloud.resize(getNumberOfElements(file, "vertex"));

            std::size_t size_x = x->buffer.size_bytes() / x->count;
            std::size_t size_y = y->buffer.size_bytes() / y->count;

            for (auto n = 0; n < cloud.size(); n++)
            {
                std::memcpy(&cloud(n).x, x->buffer.get_const() + n * size_x, size_x);
                std::memcpy(&cloud(n).y, y->buffer.get_const() + n * size_y, size_y);
            }

            if (faces)
            {
                vertices.resize(faces->count * 3);
                std::memcpy(vertices.getMemoryBlock(), faces->buffer.get_const(), faces->buffer.size_bytes());
            }

            return true;
        }

        return false;
    }

    bool read(std::istream & ifs, yarp::sig::PointCloudXYZ & cloud, yarp::sig::VectorOf<int> & vertices)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            std::shared_ptr<tinyply::PlyData> faces;

            auto x = file.request_properties_from_element("vertex", {"x"});
            auto y = file.request_properties_from_element("vertex", {"y"});
            auto z = file.request_properties_from_element("vertex", {"z"});

            try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
            catch (...) {}

            file.read(ifs);
            cloud.resize(getNumberOfElements(file, "vertex"));

            std::size_t size_x = x->buffer.size_bytes() / x->count;
            std::size_t size_y = y->buffer.size_bytes() / y->count;
            std::size_t size_z = z->buffer.size_bytes() / z->count;

            for (auto n = 0; n < cloud.size(); n++)
            {
                std::memcpy(&cloud(n).x, x->buffer.get_const() + n * size_x, size_x);
                std::memcpy(&cloud(n).y, y->buffer.get_const() + n * size_y, size_y);
                std::memcpy(&cloud(n).z, z->buffer.get_const() + n * size_z, size_z);
            }

            if (faces)
            {
                vertices.resize(faces->count * 3);
                std::memcpy(vertices.getMemoryBlock(), faces->buffer.get_const(), faces->buffer.size_bytes());
            }

            return true;
        }

        return false;
    }

    bool read(std::istream & ifs, yarp::sig::PointCloudNormal & cloud, yarp::sig::VectorOf<int> & vertices)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            std::shared_ptr<tinyply::PlyData> c, faces;
            std::size_t size_c;

            auto nx = file.request_properties_from_element("vertex", {"nx"});
            auto ny = file.request_properties_from_element("vertex", {"ny"});
            auto nz = file.request_properties_from_element("vertex", {"nz"});

            try { c = file.request_properties_from_element("vertex", {"curvature"}); }
            catch (...) {}

            try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
            catch (...) {}

            file.read(ifs);
            cloud.resize(getNumberOfElements(file, "vertex"));

            std::size_t size_nx = nx->buffer.size_bytes() / nx->count;
            std::size_t size_ny = ny->buffer.size_bytes() / ny->count;
            std::size_t size_nz = nz->buffer.size_bytes() / nz->count;

            if (c)
            {
                size_c = c->buffer.size_bytes() / c->count;
            }

            for (auto n = 0; n < cloud.size(); n++)
            {
                std::memcpy(&cloud(n).normal_x, nx->buffer.get_const() + n * size_nx, size_nx);
                std::memcpy(&cloud(n).normal_y, ny->buffer.get_const() + n * size_ny, size_ny);
                std::memcpy(&cloud(n).normal_z, nz->buffer.get_const() + n * size_nz, size_nz);

                if (c)
                {
                    std::memcpy(&cloud(n).curvature, c->buffer.get_const() + n * size_c, size_c);
                }
            }

            if (faces)
            {
                vertices.resize(faces->count * 3);
                std::memcpy(vertices.getMemoryBlock(), faces->buffer.get_const(), faces->buffer.size_bytes());
            }

            return true;
        }

        return false;
    }

    bool read(std::istream & ifs, yarp::sig::PointCloudXYZRGBA & cloud, yarp::sig::VectorOf<int> & vertices)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            std::shared_ptr<tinyply::PlyData> a, faces;
            std::size_t size_a;

            auto x = file.request_properties_from_element("vertex", {"x"});
            auto y = file.request_properties_from_element("vertex", {"y"});
            auto z = file.request_properties_from_element("vertex", {"z"});

            auto r = file.request_properties_from_element("vertex", {"red"});
            auto g = file.request_properties_from_element("vertex", {"green"});
            auto b = file.request_properties_from_element("vertex", {"blue"});

            try { a = file.request_properties_from_element("vertex", {"alpha"}); }
            catch (...) {}

            try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
            catch (...) {}

            file.read(ifs);
            cloud.resize(getNumberOfElements(file, "vertex"));

            std::size_t size_x = x->buffer.size_bytes() / x->count;
            std::size_t size_y = y->buffer.size_bytes() / y->count;
            std::size_t size_z = z->buffer.size_bytes() / z->count;

            std::size_t size_r = r->buffer.size_bytes() / r->count;
            std::size_t size_g = g->buffer.size_bytes() / g->count;
            std::size_t size_b = b->buffer.size_bytes() / b->count;

            if (a)
            {
                size_a = a->buffer.size_bytes() / a->count;
            }

            for (auto n = 0; n < cloud.size(); n++)
            {
                std::memcpy(&cloud(n).x, x->buffer.get_const() + n * size_x, size_x);
                std::memcpy(&cloud(n).y, y->buffer.get_const() + n * size_y, size_y);
                std::memcpy(&cloud(n).z, z->buffer.get_const() + n * size_z, size_z);

                std::memcpy(&cloud(n).r, r->buffer.get_const() + n * size_r, size_r);
                std::memcpy(&cloud(n).g, g->buffer.get_const() + n * size_g, size_g);
                std::memcpy(&cloud(n).b, b->buffer.get_const() + n * size_b, size_b);

                if (a)
                {
                    std::memcpy(&cloud(n).a, a->buffer.get_const() + n * size_a, size_a);
                }
            }

            if (faces)
            {
                vertices.resize(faces->count * 3);
                std::memcpy(vertices.getMemoryBlock(), faces->buffer.get_const(), faces->buffer.size_bytes());
            }

            return true;
        }

        return false;
    }

    bool read(std::istream & ifs, yarp::sig::PointCloudXYZI & cloud, yarp::sig::VectorOf<int> & vertices)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            std::shared_ptr<tinyply::PlyData> i, faces;
            std::size_t size_i;

            auto x = file.request_properties_from_element("vertex", {"x"});
            auto y = file.request_properties_from_element("vertex", {"y"});
            auto z = file.request_properties_from_element("vertex", {"z"});

            try { i = file.request_properties_from_element("vertex", {"intensity"}); }
            catch (...) {}

            try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
            catch (...) {}

            file.read(ifs);
            cloud.resize(getNumberOfElements(file, "vertex"));

            std::size_t size_x = x->buffer.size_bytes() / x->count;
            std::size_t size_y = y->buffer.size_bytes() / y->count;
            std::size_t size_z = z->buffer.size_bytes() / z->count;

            if (i)
            {
                size_i = i->buffer.size_bytes() / i->count;
            }

            for (auto n = 0; n < cloud.size(); n++)
            {
                std::memcpy(&cloud(n).x, x->buffer.get_const() + n * size_x, size_x);
                std::memcpy(&cloud(n).y, y->buffer.get_const() + n * size_y, size_y);
                std::memcpy(&cloud(n).z, z->buffer.get_const() + n * size_z, size_z);

                if (i)
                {
                    std::memcpy(&cloud(n).intensity, i->buffer.get_const() + n * size_i, size_i);
                }
            }

            if (faces)
            {
                vertices.resize(faces->count * 3);
                std::memcpy(vertices.getMemoryBlock(), faces->buffer.get_const(), faces->buffer.size_bytes());
            }

            return true;
        }

        return false;
    }

    bool read(std::istream & ifs, yarp::sig::PointCloudInterestPointXYZ & cloud, yarp::sig::VectorOf<int> & vertices)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            std::shared_ptr<tinyply::PlyData> s, faces;
            std::size_t size_s;

            auto x = file.request_properties_from_element("vertex", {"x"});
            auto y = file.request_properties_from_element("vertex", {"y"});
            auto z = file.request_properties_from_element("vertex", {"z"});

            try { s = file.request_properties_from_element("vertex", {"strength"}); }
            catch (...) {}

            try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
            catch (...) {}

            file.read(ifs);
            cloud.resize(getNumberOfElements(file, "vertex"));

            std::size_t size_x = x->buffer.size_bytes() / x->count;
            std::size_t size_y = y->buffer.size_bytes() / y->count;
            std::size_t size_z = z->buffer.size_bytes() / z->count;

            if (s)
            {
                size_s = s->buffer.size_bytes() / s->count;
            }

            for (auto n = 0; n < cloud.size(); n++)
            {
                std::memcpy(&cloud(n).x, x->buffer.get_const() + n * size_x, size_x);
                std::memcpy(&cloud(n).y, y->buffer.get_const() + n * size_y, size_y);
                std::memcpy(&cloud(n).z, z->buffer.get_const() + n * size_z, size_z);

                if (s)
                {
                    std::memcpy(&cloud(n).strength, s->buffer.get_const() + n * size_s, size_s);
                }
            }

            if (faces)
            {
                vertices.resize(faces->count * 3);
                std::memcpy(vertices.getMemoryBlock(), faces->buffer.get_const(), faces->buffer.size_bytes());
            }

            return true;
        }

        return false;
    }

    bool read(std::istream & ifs, yarp::sig::PointCloudXYZNormal & cloud, yarp::sig::VectorOf<int> & vertices)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            std::shared_ptr<tinyply::PlyData> c, faces;
            std::size_t size_c;

            auto x = file.request_properties_from_element("vertex", {"x"});
            auto y = file.request_properties_from_element("vertex", {"y"});
            auto z = file.request_properties_from_element("vertex", {"z"});

            auto nx = file.request_properties_from_element("vertex", {"nx"});
            auto ny = file.request_properties_from_element("vertex", {"ny"});
            auto nz = file.request_properties_from_element("vertex", {"nz"});

            try { c = file.request_properties_from_element("vertex", {"curvature"}); }
            catch (...) {}

            try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
            catch (...) {}

            file.read(ifs);
            cloud.resize(getNumberOfElements(file, "vertex"));

            std::size_t size_x = x->buffer.size_bytes() / x->count;
            std::size_t size_y = y->buffer.size_bytes() / y->count;
            std::size_t size_z = z->buffer.size_bytes() / z->count;

            std::size_t size_nx = nx->buffer.size_bytes() / nx->count;
            std::size_t size_ny = ny->buffer.size_bytes() / ny->count;
            std::size_t size_nz = nz->buffer.size_bytes() / nz->count;

            if (c)
            {
                size_c = c->buffer.size_bytes() / c->count;
            }

            for (auto n = 0; n < cloud.size(); n++)
            {
                std::memcpy(&cloud(n).x, x->buffer.get_const() + n * size_x, size_x);
                std::memcpy(&cloud(n).y, y->buffer.get_const() + n * size_y, size_y);
                std::memcpy(&cloud(n).z, z->buffer.get_const() + n * size_z, size_z);

                std::memcpy(&cloud(n).normal_x, nx->buffer.get_const() + n * size_nx, size_nx);
                std::memcpy(&cloud(n).normal_y, ny->buffer.get_const() + n * size_ny, size_ny);
                std::memcpy(&cloud(n).normal_z, nz->buffer.get_const() + n * size_nz, size_nz);

                if (c)
                {
                    std::memcpy(&cloud(n).curvature, c->buffer.get_const() + n * size_c, size_c);
                }
            }

            if (faces)
            {
                vertices.resize(faces->count * 3);
                std::memcpy(vertices.getMemoryBlock(), faces->buffer.get_const(), faces->buffer.size_bytes());
            }

            return true;
        }

        return false;
    }

    bool read(std::istream & ifs, yarp::sig::PointCloudXYZNormalRGBA & cloud, yarp::sig::VectorOf<int> & vertices)
    {
        tinyply::PlyFile file;

        if (file.parse_header(ifs))
        {
            std::shared_ptr<tinyply::PlyData> c, a, faces;
            std::size_t size_c, size_a;

            auto x = file.request_properties_from_element("vertex", {"x"});
            auto y = file.request_properties_from_element("vertex", {"y"});
            auto z = file.request_properties_from_element("vertex", {"z"});

            auto nx = file.request_properties_from_element("vertex", {"nx"});
            auto ny = file.request_properties_from_element("vertex", {"ny"});
            auto nz = file.request_properties_from_element("vertex", {"nz"});

            try { c = file.request_properties_from_element("vertex", {"curvature"}); }
            catch (...) {}

            auto r = file.request_properties_from_element("vertex", {"red"});
            auto g = file.request_properties_from_element("vertex", {"green"});
            auto b = file.request_properties_from_element("vertex", {"blue"});

            try { a = file.request_properties_from_element("vertex", {"alpha"}); }
            catch (...) {}

            try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
            catch (...) {}

            file.read(ifs);
            cloud.resize(getNumberOfElements(file, "vertex"));

            std::size_t size_x = x->buffer.size_bytes() / x->count;
            std::size_t size_y = y->buffer.size_bytes() / y->count;
            std::size_t size_z = z->buffer.size_bytes() / z->count;

            std::size_t size_nx = nx->buffer.size_bytes() / nx->count;
            std::size_t size_ny = ny->buffer.size_bytes() / ny->count;
            std::size_t size_nz = nz->buffer.size_bytes() / nz->count;

            if (c)
            {
                size_c = c->buffer.size_bytes() / c->count;
            }

            std::size_t size_r = r->buffer.size_bytes() / r->count;
            std::size_t size_g = g->buffer.size_bytes() / g->count;
            std::size_t size_b = b->buffer.size_bytes() / b->count;

            if (a)
            {
                size_a = a->buffer.size_bytes() / a->count;
            }

            for (auto n = 0; n < cloud.size(); n++)
            {
                std::memcpy(&cloud(n).x, x->buffer.get_const() + n * size_x, size_x);
                std::memcpy(&cloud(n).y, y->buffer.get_const() + n * size_y, size_y);
                std::memcpy(&cloud(n).z, z->buffer.get_const() + n * size_z, size_z);

                std::memcpy(&cloud(n).normal_x, nx->buffer.get_const() + n * size_nx, size_nx);
                std::memcpy(&cloud(n).normal_y, ny->buffer.get_const() + n * size_ny, size_ny);
                std::memcpy(&cloud(n).normal_z, nz->buffer.get_const() + n * size_nz, size_nz);

                if (c)
                {
                    std::memcpy(&cloud(n).curvature, c->buffer.get_const() + n * size_c, size_c);
                }

                std::memcpy(&cloud(n).r, r->buffer.get_const() + n * size_r, size_r);
                std::memcpy(&cloud(n).g, g->buffer.get_const() + n * size_g, size_g);
                std::memcpy(&cloud(n).b, b->buffer.get_const() + n * size_b, size_b);

                if (a)
                {
                    std::memcpy(&cloud(n).a, a->buffer.get_const() + n * size_a, size_a);
                }
            }

            if (faces)
            {
                vertices.resize(faces->count * 3);
                std::memcpy(vertices.getMemoryBlock(), faces->buffer.get_const(), faces->buffer.size_bytes());
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
    return savePLY(filename, cloud, {}, isBinary);
}

template <typename T>
bool savePLY(const std::string & filename, const yarp::sig::PointCloud<T> & cloud, const yarp::sig::VectorOf<int> & vertices, bool isBinary)
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
        write(os, cloud, vertices, isBinary);
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
    yarp::sig::VectorOf<int> vertices;
    return loadPLY(filename, cloud, vertices);
}

template <typename T>
bool loadPLY(const std::string & filename, yarp::sig::PointCloud<T> & cloud, yarp::sig::VectorOf<int> & vertices)
{
    std::ifstream ifs(filename, std::ios::binary);

    if (ifs.fail())
    {
        yError() << "unable to open" << filename << "for read";
        return false;
    }

    std::vector<std::uint8_t> fileBufferBytes;
    ifs.seekg(0, std::ios::end);
    std::size_t sizeBytes = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    fileBufferBytes.resize(sizeBytes);

    if (!ifs.read((char *)fileBufferBytes.data(), sizeBytes))
    {
        yError() << "unable to read from" << filename;
        return false;
    }

    memory_stream ms((char *)fileBufferBytes.data(), fileBufferBytes.size());

    try
    {
        return read(ms, cloud, vertices);
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
