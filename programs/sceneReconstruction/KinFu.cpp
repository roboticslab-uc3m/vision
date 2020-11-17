// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinectFusionAdapter.hpp"

#include <algorithm>
#include <map>
#include <iterator>
#include <sstream>
#include <type_traits>

#include <opencv2/core/version.hpp>
#include <opencv2/rgbd/kinfu.hpp>

#include <yarp/cv/Cv.h>

#include <ColorDebug.h>

namespace
{
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

    template <typename T>
    void updateParam(cv::kinfu::Params & params, T cv::kinfu::Params::* param, const yarp::os::Searchable & config,
        const std::string & name, const std::string & description)
    {
        std::stringstream ss;
        ss << name;

        if (config.check(name, description))
        {
            params.*param = getValue<T>(config.find(name));
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

#if CV_VERSION_MINOR >= 4
    std::map<std::string, cv::kinfu::VolumeType> stringToCvVolume {
        {"tsdf", cv::kinfu::VolumeType::TSDF},
        {"hashtsdf", cv::kinfu::VolumeType::HASHTSDF}
    };
#endif
}

namespace roboticslab
{

class KinFu : public KinectFusionAdapter
{
public:
    KinFu(const cv::Ptr<cv::kinfu::Params> & params) : handle(cv::kinfu::KinFu::create(params))
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
    cv::Ptr<cv::kinfu::KinFu> handle;
};

std::unique_ptr<KinectFusionAdapter> makeKinFu(const yarp::os::Searchable & config, const yarp::sig::IntrinsicParams & intrinsic, int width, int height)
{
    using Params = cv::kinfu::Params;

    auto params = Params::defaultParams();

    params->frameSize = cv::Size(width, height);
    CD_INFO("dimensions: width = %d, height = %d\n", width, height);

    float fx = intrinsic.focalLengthX;
    float fy = intrinsic.focalLengthY;
    float cx = intrinsic.principalPointX;
    float cy = intrinsic.principalPointY;

    params->intr = cv::Matx33f(fx,  0, cx,
                                0, fy, cy,
                                0,  0,  1);

    CD_INFO("intrinsic params: fx = %f, fy = %f, cx = %f, cy = %f\n", fx, fy, cx, cy);

    updateParam(*params, &Params::depthFactor, config, "depthFactor", "pre-scale per 1 meter for input values");
    updateParam(*params, &Params::voxelSize, config, "voxelSize", "size of voxel in meters");
    updateParam(*params, &Params::raycast_step_factor, config, "raycastStepFactor", "a length in voxel sizes for one raycast step");

    if (config.check("volumeDims", "resolution of voxel space"))
    {
        yarp::os::Bottle * volumeDims = config.find("volumeDims").asList();

        if (volumeDims == nullptr || volumeDims->size() != 3)
        {
            CD_ERROR("Parameter volumeDims must be a list of 3 integers.\n");
            return nullptr;
        }

        params->volumeDims = cv::Vec3i(volumeDims->get(0).asInt32(), volumeDims->get(1).asInt32(), volumeDims->get(2).asInt32());
        CD_INFO("volumeDims: %s\n", volumeDims->toString().c_str());
    }
    else
    {
        const auto & cvDims = params->volumeDims;
        CD_INFO("volumeDims (DEFAULT): (%d %d %d)\n", cvDims[0], cvDims[1], cvDims[2]);
    }

    if (config.check("lightPose", "light pose for rendering in meters"))
    {
        yarp::os::Bottle * lightPose = config.find("lightPose").asList();

        if (lightPose == nullptr || lightPose->size() != 3)
        {
            CD_ERROR("Parameter lightPose must be a list of 3 floats.\n");
            return nullptr;
        }

        params->lightPose = cv::Vec3f(lightPose->get(0).asFloat32(), lightPose->get(1).asFloat32(), lightPose->get(2).asFloat32());
        CD_INFO("lightPose: %s\n", lightPose->toString().c_str());
    }
    else
    {
        const auto & cvLight = params->lightPose;
        CD_INFO("lightPose (DEFAULT): (%f %f %f)\n", cvLight[0], cvLight[1], cvLight[2]);
    }

    if (config.check("volumePoseTransl", "volume pose (translation vector) in meters"))
    {
        yarp::os::Bottle * volumePoseTransl = config.find("volumePoseTransl").asList();

        if (volumePoseTransl == nullptr || volumePoseTransl->size() != 3)
        {
            CD_ERROR("Parameter volumePoseTransl must be a list of 3 floats.\n");
            return nullptr;
        }

        auto transl = cv::Vec3f(volumePoseTransl->get(0).asFloat32(), volumePoseTransl->get(1).asFloat32(), volumePoseTransl->get(2).asFloat32());
        params->volumePose.translation(transl);
        CD_INFO("volumePoseTransl: %s\n", volumePoseTransl->toString().c_str());
    }
    else
    {
        const auto & transl = params->volumePose.translation();
        CD_INFO("volumePoseTransl (DEFAULT): (%f %f %f)\n", transl[0], transl[1], transl[2]);
    }

    if (config.check("volumePoseRot", "volume pose (rotation matrix) in radians"))
    {
        yarp::os::Bottle * volumePoseRot = config.find("volumePoseRot").asList();

        if (volumePoseRot == nullptr || volumePoseRot->size() != 9)
        {
            CD_ERROR("Parameter volumePoseRot must be a list of 9 floats (3x3 matrix).\n");
            return nullptr;
        }

        auto rot = cv::Matx33f(volumePoseRot->get(0).asFloat32(), volumePoseRot->get(1).asFloat32(), volumePoseRot->get(2).asFloat32(),
                               volumePoseRot->get(3).asFloat32(), volumePoseRot->get(4).asFloat32(), volumePoseRot->get(5).asFloat32(),
                               volumePoseRot->get(6).asFloat32(), volumePoseRot->get(7).asFloat32(), volumePoseRot->get(8).asFloat32());

        params->volumePose.rotation(rot);
        CD_INFO("volumePoseRot: %s\n", volumePoseRot->toString().c_str());
    }
    else
    {
        const auto & rot = params->volumePose.rotation();
        CD_INFO("volumePoseRot (DEFAULT): (%f %f %f %f %f %f %f %f %f)\n", rot(0,0), rot(0,1), rot(0,2), rot(1,0), rot(1,1), rot(1,2), rot(2,0), rot(2,1), rot(2,2));
    }

#if CV_VERSION_MINOR >= 2
    updateParam(*params, &Params::truncateThreshold, config, "truncateThreshold", "threshold for depth truncation in meters");
#endif

#if CV_VERSION_MINOR >= 4
    if (config.check("volumeType", "type of voxel volume (tsdf, hashtsdf)"))
    {
        std::string volumeType = config.find("volumeType").asString();

        if (stringToCvVolume.find(volumeType) == stringToCvVolume.end())
        {
            CD_ERROR("Unsupported volume type: %s.\n", volumeType.c_str());
            return nullptr;
        }

        params->volumeType = stringToCvVolume[volumeType];
        CD_INFO("volumeType: %s\n", volumeType.c_str());
    }
    else
    {
        auto res = std::find_if(stringToCvVolume.begin(), stringToCvVolume.end(), [&params](const auto & el) { return el.second == params->volumeType; });
        CD_INFO("volumeType (DEFAULT): %s\n", res->first.c_str());
    }
#endif

    updateParam(*params, &Params::bilateral_kernel_size, config, "bilateralKernelSize", "kernel size in pixels for bilateral smooth");
    updateParam(*params, &Params::bilateral_sigma_depth, config, "bilateralSigmaDepth", "depth sigma in meters for bilateral smooth");
    updateParam(*params, &Params::bilateral_sigma_spatial, config, "bilateralSigmaSpatial", "spatial sigma in pixels for bilateral smooth");

    updateParam(*params, &Params::icpAngleThresh, config, "icpAngleThresh", "angle threshold in radians for ICP");
    updateParam(*params, &Params::icpDistThresh, config, "icpDistThresh", "distance threshold in meters for ICP");
    updateParam(*params, &Params::pyramidLevels, config, "pyramidLevels", "number of pyramid levels for ICP");

    if (config.check("icpIterations", "list of iterations per each ICP level"))
    {
        yarp::os::Bottle * icpIterations = config.find("icpIterations").asList();

        if (icpIterations == nullptr)
        {
            CD_ERROR("Parameter icpIterations must be a list.\n");
            return nullptr;
        }

        for (auto i = 0; i < icpIterations->size(); i++)
        {
            params->icpIterations.push_back(icpIterations->get(i).asInt32());
        }

        CD_INFO("icpIterations: %s\n", icpIterations->toString().c_str());
    }
    else
    {
        CD_INFO("icpIterations (DEFAULT): (%s)\n", vectorToString(params->icpIterations).c_str());
    }

    updateParam(*params, &Params::tsdf_max_weight, config, "tsdfMaxWeight", "max number of frames per voxel");
    updateParam(*params, &Params::tsdf_min_camera_movement, config, "tsdfMinCameraMovement", "minimal camera movement in meters");
    updateParam(*params, &Params::tsdf_trunc_dist, config, "tsdfTruncDist", "distance to truncate in meters");

    return std::make_unique<KinFu>(params);
}

} // namespace roboticslab
