// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KINECT_FUSION_IMPL_HPP__
#define __KINECT_FUSION_IMPL_HPP__

#include "KinectFusion.hpp"

#include <mutex>
#include <type_traits>

#include <yarp/os/LogStream.h>
#include <yarp/cv/Cv.h>

#include "LogComponent.hpp"

namespace roboticslab
{

template <typename T>
class KinectFusionImpl : public KinectFusion
{
public:
    KinectFusionImpl(const cv::Ptr<T> & other) : handle(other)
    {
        cv::setUseOptimized(true);
    }

    void getCloud(yarp::sig::PointCloudXYZNormalRGBA & cloudWithNormals) const override
    {
        cv::UMat points, normals;

        mtx.lock();
        handle->getCloud(points, normals);
        mtx.unlock();

        cv::Mat _points = points.getMat(cv::ACCESS_FAST); // no memcpy
        cv::Mat _normals = normals.getMat(cv::ACCESS_FAST); // no memcpy

        cloudWithNormals.resize(points.rows);

        for (auto i = 0; i < points.rows; i++)
        {
            const auto & point = _points.at<cv::Vec4f>(i);
            const auto & normal = _normals.at<cv::Vec4f>(i);
            cloudWithNormals(i) = {{point[0], point[1], point[2]}, {normal[0], normal[1], normal[2]}, 0};
        }
    }

    void getPoints(yarp::sig::PointCloudXYZ & cloud) const override
    {
        cv::UMat points;

        mtx.lock();
        handle->getPoints(points);
        mtx.unlock();

        cv::Mat _points = points.getMat(cv::ACCESS_FAST); // no memcpy
        auto data = const_cast<const char *>(reinterpret_cast<char *>(_points.data));

        cloud.fromExternalPC(data, yarp::sig::PointCloudBasicType::PC_XYZ_DATA, _points.rows, 1);
    }

    void getPose(yarp::sig::Matrix & pose) const override
    {
        mtx.lock();
        const auto & affine = handle->getPose().matrix;
        mtx.unlock();

        pose.resize(4, 4);

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                pose(i, j) = affine(i, j);
            }
        }
    }

    bool update(const yarp::sig::ImageOf<yarp::sig::PixelFloat> & depthFrame, const yarp::sig::FlexImage & colorFrame) override
    {
        // Cast away constness so that toCvMat accepts the YARP image. This function
        // does not alter the inner structure of PixelFloat images anyway.
        auto & nonConstDepthFrame = const_cast<yarp::sig::ImageOf<yarp::sig::PixelFloat> &>(depthFrame);
        cv::Mat mat = yarp::cv::toCvMat(nonConstDepthFrame);

        cv::UMat umat;
        mat.convertTo(umat, mat.type(), 1000.0); // OpenCV uses milimeters

        std::lock_guard<std::mutex> lock(mtx);
        return handle->update(umat);
    }

    void reset() override
    {
        std::lock_guard<std::mutex> lock(mtx);
        handle->reset();
    }

    void render(yarp::sig::FlexImage & image) const override
    {
        cv::UMat umat;

        mtx.lock();
        handle->render(umat);
        mtx.unlock();

        cv::Mat mat = umat.getMat(cv::ACCESS_FAST); // no memcpy
        const auto & bgr = yarp::cv::fromCvMat<yarp::sig::PixelBgra>(mat); // no conversion
        image.copy(bgr); // bgra to grayscale/rgb (single step convert+assign)
    }

private:
    cv::Ptr<T> handle;
    mutable std::mutex mtx;
};

template <typename T>
T getValue(const yarp::os::Value & v)
{
    if constexpr (std::is_integral_v<T>)
    {
        return v.asInt32();
    }
    else if constexpr (std::is_floating_point_v<T>)
    {
        return v.asFloat64();
    }
    else
    {
        // https://stackoverflow.com/a/64354296/10404307
        static_assert(!sizeof(T), "Unsupported type");
    }
}

template <typename TParams, typename TRet>
void updateParam(TParams & params, TRet TParams::* param, const yarp::os::Searchable & config,
                 const std::string & name, const std::string & description)
{
    auto && log = yCInfo(KINFU);
    log << name + ":";

    if (config.check(name, description))
    {
        params.*param = getValue<TRet>(config.find(name));
    }
    else
    {
        log << "(DEFAULT):";
    }

    log << params.*param;
}

} // namespace roboticslab

#endif // __KINECT_FUSION_IMPL_HPP__
