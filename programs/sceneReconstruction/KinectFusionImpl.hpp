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
        cv::Mat points, normals;

        mtx.lock();
        handle->getCloud(points, normals);
        mtx.unlock();

        cloudWithNormals.resize(points.rows);

        for (auto i = 0; i < points.rows; i++)
        {
            const auto & point = points.at<cv::Vec4f>(i);
            const auto & normal = normals.at<cv::Vec4f>(i);
            cloudWithNormals(i) = {{point[0], point[1], point[2]}, {normal[0], normal[1], normal[2]}, 0};
        }
    }

    void getPoints(yarp::sig::PointCloudXYZ & cloud) const override
    {
        cv::Mat points;

        mtx.lock();
        handle->getPoints(points);
        mtx.unlock();

        cloud.resize(points.rows);

        for (auto i = 0; i < points.rows; i++)
        {
            const auto & point = points.at<cv::Vec4f>(i);
            cloud(i) = {point[0], point[1], point[2]};
        }
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

    bool update(const yarp::sig::ImageOf<yarp::sig::PixelFloat> & depthFrame, const yarp::sig::FlexImage & rgbFrame) override
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

    void render(yarp::sig::ImageOf<yarp::sig::PixelMono> & image) const override
    {
        image.copy(renderBgra()); // bgra to grayscale (single step convert+assign)
    }

    void render(yarp::sig::ImageOf<yarp::sig::PixelRgb> & image) const override
    {
        image.copy(renderBgra()); // bgra to rgba (single step convert+assign)
    }

private:
    yarp::sig::ImageOf<yarp::sig::PixelBgra> renderBgra() const
    {
        cv::UMat umat;

        mtx.lock();
        handle->render(umat);
        mtx.unlock();

        cv::Mat mat = umat.getMat(cv::ACCESS_FAST); // no memcpy
        return yarp::cv::fromCvMat<yarp::sig::PixelBgra>(mat); // no conversion, use RVO
    }

    cv::Ptr<T> handle;
    mutable std::mutex mtx;
};

template <typename T>
inline std::enable_if_t<std::is_integral<T>::value, T>
getValue(const yarp::os::Value & v)
{
    return v.asInt32();
}

template <typename T>
inline std::enable_if_t<std::is_floating_point<T>::value, T>
getValue(const yarp::os::Value & v)
{
    return v.asFloat32();
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
