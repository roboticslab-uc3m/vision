// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KINECT_FUSION_IMPL_HPP__
#define __KINECT_FUSION_IMPL_HPP__

#include "KinectFusion.hpp"

#include <algorithm>
#include <iterator>
#include <sstream>
#include <type_traits>

#include <yarp/cv/Cv.h>

#include <ColorDebug.h>

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

    void getCloud(yarp::sig::PointCloudXYZNormal & cloudWithNormals) const override
    {
        cv::Mat points, normals;
        handle->getCloud(points, normals);
        cloudWithNormals.resize(points.rows);

        for (auto i = 0; i < points.rows; i++)
        {
            const auto & point = points.at<cv::Vec4f>(i);
            const auto & normal = normals.at<cv::Vec4f>(i);
            cloudWithNormals(i) = {{point[0], point[1], point[2]}, {normal[0], normal[1], normal[2]}};
        }
    }

    void getPoints(yarp::sig::PointCloudXYZ & cloud) const override
    {
        cv::Mat points;
        handle->getPoints(points);
        cloud.resize(points.rows);

        for (auto i = 0; i < points.rows; i++)
        {
            const auto & point = points.at<cv::Vec4f>(i);
            cloud(i) = {point[0], point[1], point[2]};
        }
    }

    void getPose(yarp::sig::Matrix & pose) const override
    {
        const auto & affine = handle->getPose().matrix;
        pose.resize(4, 4);

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                pose(i, j) = affine(i, j);
            }
        }
    }

    bool update(const yarp::sig::ImageOf<yarp::sig::PixelFloat> & depthFrame) override
    {
        // Cast away constness so that toCvMat accepts the YARP image. This function
        // does not alter the inner structure of PixelFloat images anyway.
        auto & nonConstDepthFrame = const_cast<yarp::sig::ImageOf<yarp::sig::PixelFloat> &>(depthFrame);
        cv::Mat mat = yarp::cv::toCvMat(nonConstDepthFrame);
        cv::UMat umat;
        mat.convertTo(umat, mat.type(), 1000.0); // OpenCV uses milimeters
        return handle->update(umat);
    }

    void reset() override
    {
        handle->reset();
    }

    void render(yarp::sig::ImageOf<yarp::sig::PixelMono> & image) const override
    {
        cv::UMat umat;
        handle->render(umat);
        cv::Mat mat = umat.getMat(cv::ACCESS_FAST); // no memcpy
        const auto & temp = yarp::cv::fromCvMat<yarp::sig::PixelBgra>(mat); // no conversion
        image.copy(temp); // bgra to grayscale (single step convert+assign)
    }

private:
    cv::Ptr<T> handle;
};

template <typename T, std::enable_if_t<std::is_integral<T>::value, bool> = true>
inline T getValue(const yarp::os::Value & v)
{
    return v.asInt32();
}

template <typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
inline T getValue(const yarp::os::Value & v)
{
    return v.asFloat32();
}

template <typename TParams, typename TRet>
void updateParam(TParams & params, TRet TParams::* param, const yarp::os::Searchable & config,
    const std::string & name, const std::string & description)
{
    std::stringstream ss;
    ss << name;

    if (config.check(name, description))
    {
        params.*param = getValue<TRet>(config.find(name));
        ss << ": ";
    }
    else
    {
        ss << " (DEFAULT): ";
    }

    ss << params.*param << "\n";
    CD_INFO(ss.str().c_str());
}

template <typename T>
inline std::string vectorToString(const std::vector<T> & vec, const std::string & delimiter = " ")
{
    // https://stackoverflow.com/a/8581865
    std::ostringstream oss;
    std::copy(vec.begin(), vec.end() - 1, std::ostream_iterator<T>(oss, delimiter.c_str()));
    oss << vec.back();
    return oss.str();
}

} // namespace roboticslab

#endif // __KINECT_FUSION_IMPL_HPP__
