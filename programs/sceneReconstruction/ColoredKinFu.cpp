// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinectFusionImpl.hpp"

#include <map>
#include <yarp/os/LogStream.h>
#include <opencv2/rgbd/colored_kinfu.hpp>

#include "LogComponent.hpp"

using namespace roboticslab;

template <>
void KinectFusionImpl<cv::colored_kinfu::ColoredKinFu>::getCloud(yarp::sig::PointCloudXYZNormalRGBA & cloudWithNormals) const
{
    cv::Mat points, normals, colors;

    mtx.lock();
    handle->getCloud(points, normals, colors);
    mtx.unlock();

    cloudWithNormals.resize(points.rows);

    for (auto i = 0; i < points.rows; i++)
    {
        using color_t = unsigned char;
        const auto & point = points.at<cv::Vec4f>(i);
        const auto & normal = normals.at<cv::Vec4f>(i);
        const auto & color = colors.at<cv::Vec4f>(i);

        cloudWithNormals(i) = {
            {point[0], point[1], point[2]},
            {normal[0], normal[1], normal[2]},
            {static_cast<color_t>(color[0]), static_cast<color_t>(color[1]), static_cast<color_t>(color[2]), static_cast<color_t>(color[3])}
        };
    }
}

template <>
bool KinectFusionImpl<cv::colored_kinfu::ColoredKinFu>::update(const yarp::sig::ImageOf<yarp::sig::PixelFloat> & depthFrame,
                                                               const yarp::sig::FlexImage & rgbFrame)
{
    // Cast away constness so that toCvMat accepts the YARP image. This function
    // does not alter the inner structure of PixelFloat images anyway.
    auto & nonConstDepthFrame = const_cast<yarp::sig::ImageOf<yarp::sig::PixelFloat> &>(depthFrame);
    cv::Mat depthMat = yarp::cv::toCvMat(nonConstDepthFrame);

    yarp::sig::ImageOf<yarp::sig::PixelBgr> bgrFrame;
    bgrFrame.copy(rgbFrame);
    cv::Mat bgrMat = yarp::cv::toCvMat(bgrFrame);

    cv::UMat depthUmat;
    depthMat.convertTo(depthUmat, depthMat.type(), 1000.0); // OpenCV uses milimeters

    std::lock_guard<std::mutex> lock(mtx);
    return handle->update(depthUmat, bgrMat);
}

namespace
{
    std::map<std::string, cv::kinfu::VolumeType> stringToCvVolume {
        {"tsdf", cv::kinfu::VolumeType::TSDF},
        {"hashtsdf", cv::kinfu::VolumeType::HASHTSDF},
        {"coloredtsdf", cv::kinfu::VolumeType::COLOREDTSDF}
    };
}

namespace roboticslab
{

std::unique_ptr<KinectFusion> makeColoredKinFu(const yarp::os::Searchable & config,
                                               const yarp::sig::IntrinsicParams & depthIntrinsic,
                                               const yarp::sig::IntrinsicParams & rgbIntrinsic,
                                               int depthWidth, int depthHeight,
                                               int rgbWidth, int rgbHeight)
{
    using Params = cv::colored_kinfu::Params;

    auto params = Params::defaultParams();

    yCInfo(KINFU) << "--- CAMERA PARAMETERS (depth) ---";

    params->frameSize = cv::Size(depthWidth, depthHeight);
    yCInfo(KINFU) << "width:" << depthWidth;
    yCInfo(KINFU) << "height:" << depthHeight;

    params->intr = cv::Matx33f(depthIntrinsic.focalLengthX,                           0, depthIntrinsic.principalPointX,
                                                         0, depthIntrinsic.focalLengthY, depthIntrinsic.principalPointY,
                                                         0,                           0,                              1);

    yCInfo(KINFU) << "focal length (X):" << depthIntrinsic.focalLengthX;
    yCInfo(KINFU) << "focal length (Y):" << depthIntrinsic.focalLengthY;
    yCInfo(KINFU) << "principal point (X):" << depthIntrinsic.principalPointX;
    yCInfo(KINFU) << "principal point (Y):" << depthIntrinsic.principalPointY;

    yCInfo(KINFU) << "--- CAMERA PARAMETERS (RGB) ---";

    params->rgb_frameSize = cv::Size(rgbWidth, rgbHeight);
    yCInfo(KINFU) << "width:" << rgbWidth;
    yCInfo(KINFU) << "height:" << rgbHeight;

    params->rgb_intr = cv::Matx33f(rgbIntrinsic.focalLengthX,                         0, rgbIntrinsic.principalPointX,
                                                           0, rgbIntrinsic.focalLengthY, rgbIntrinsic.principalPointY,
                                                           0,                         0,                            1);

    yCInfo(KINFU) << "focal length (X):" << rgbIntrinsic.focalLengthX;
    yCInfo(KINFU) << "focal length (Y):" << rgbIntrinsic.focalLengthY;
    yCInfo(KINFU) << "principal point (X):" << rgbIntrinsic.principalPointX;
    yCInfo(KINFU) << "principal point (Y):" << rgbIntrinsic.principalPointY;

    yCInfo(KINFU) << "--- ALGORITHM PARAMETERS ---";

    yCInfo(KINFU) << "algorithm: colored_kinfu";

    updateParam(*params, &Params::bilateral_kernel_size, config, "bilateralKernelSize", "kernel size in pixels for bilateral smooth");
    updateParam(*params, &Params::bilateral_sigma_depth, config, "bilateralSigmaDepth", "depth sigma in meters for bilateral smooth");
    updateParam(*params, &Params::bilateral_sigma_spatial, config, "bilateralSigmaSpatial", "spatial sigma in pixels for bilateral smooth");
    updateParam(*params, &Params::depthFactor, config, "depthFactor", "pre-scale per 1 meter for input values");
    updateParam(*params, &Params::icpAngleThresh, config, "icpAngleThresh", "angle threshold in radians for ICP");
    updateParam(*params, &Params::icpDistThresh, config, "icpDistThresh", "distance threshold in meters for ICP");

    if (config.check("icpIterations", "list of iterations per each ICP level"))
    {
        yarp::os::Bottle * icpIterations = config.find("icpIterations").asList();

        if (icpIterations == nullptr)
        {
            yCError(KINFU) << "Parameter icpIterations must be a list";
            return nullptr;
        }

        params->icpIterations.resize(icpIterations->size());

        for (auto i = 0; i < icpIterations->size(); i++)
        {
            params->icpIterations[i] = icpIterations->get(i).asInt32();
        }

        yCInfo(KINFU) << "icpIterations:" << icpIterations->toString();
    }
    else
    {
        yCInfo(KINFU) << "icpIterations (DEFAULT):" << params->icpIterations;
    }

    if (config.check("lightPose", "light pose for rendering in meters"))
    {
        yarp::os::Bottle * lightPose = config.find("lightPose").asList();

        if (lightPose == nullptr || lightPose->size() != 3)
        {
            yCError(KINFU) << "Parameter lightPose must be a list of 3 floats";
            return nullptr;
        }

        params->lightPose = cv::Vec3f(lightPose->get(0).asFloat32(), lightPose->get(1).asFloat32(), lightPose->get(2).asFloat32());
        yCInfo(KINFU) << "lightPose:" << lightPose->toString();
    }
    else
    {
        const auto & cvLight = params->lightPose;
        yCInfo(KINFU) << "lightPose (DEFAULT):" << cvLight[0] << cvLight[1] << cvLight[2];
    }

    updateParam(*params, &Params::pyramidLevels, config, "pyramidLevels", "number of pyramid levels for ICP");
    updateParam(*params, &Params::raycast_step_factor, config, "raycastStepFactor", "a length in voxel sizes for one raycast step");
    updateParam(*params, &Params::truncateThreshold, config, "truncateThreshold", "threshold for depth truncation in meters");
    updateParam(*params, &Params::tsdf_max_weight, config, "tsdfMaxWeight", "max number of frames per voxel");
    updateParam(*params, &Params::tsdf_min_camera_movement, config, "tsdfMinCameraMovement", "minimal camera movement in meters");
    updateParam(*params, &Params::tsdf_trunc_dist, config, "tsdfTruncDist", "distance to truncate in meters");

    if (config.check("volumeDims", "resolution of voxel space"))
    {
        yarp::os::Bottle * volumeDims = config.find("volumeDims").asList();

        if (volumeDims == nullptr || volumeDims->size() != 3)
        {
            yCError(KINFU) << "Parameter volumeDims must be a list of 3 integers";
            return nullptr;
        }

        params->volumeDims = cv::Vec3i(volumeDims->get(0).asInt32(), volumeDims->get(1).asInt32(), volumeDims->get(2).asInt32());
        yCInfo(KINFU) << "volumeDims:" << volumeDims->toString();
    }
    else
    {
        const auto & cvDims = params->volumeDims;
        yCInfo(KINFU) << "volumeDims (DEFAULT):" << cvDims[0] << cvDims[1] << cvDims[2];
    }

    if (config.check("volumePoseRot", "volume pose (rotation matrix) in radians"))
    {
        yarp::os::Bottle * volumePoseRot = config.find("volumePoseRot").asList();

        if (volumePoseRot == nullptr || volumePoseRot->size() != 9)
        {
            yCError(KINFU) << "Parameter volumePoseRot must be a list of 9 floats (3x3 matrix)";
            return nullptr;
        }

        auto rot = cv::Matx33f(volumePoseRot->get(0).asFloat32(), volumePoseRot->get(1).asFloat32(), volumePoseRot->get(2).asFloat32(),
                               volumePoseRot->get(3).asFloat32(), volumePoseRot->get(4).asFloat32(), volumePoseRot->get(5).asFloat32(),
                               volumePoseRot->get(6).asFloat32(), volumePoseRot->get(7).asFloat32(), volumePoseRot->get(8).asFloat32());

        params->volumePose.rotation(rot);
        yCInfo(KINFU) << "volumePoseRot:" << volumePoseRot->toString();
    }
    else
    {
        const auto & rot = params->volumePose.rotation();
        yCInfo(KINFU) << "volumePoseRot (DEFAULT):" << rot(0,0) << rot(0,1) << rot(0,2) << rot(1,0) << rot(1,1) << rot(1,2) << rot(2,0) << rot(2,1) << rot(2,2);
    }

    if (config.check("volumePoseTransl", "volume pose (translation vector) in meters"))
    {
        yarp::os::Bottle * volumePoseTransl = config.find("volumePoseTransl").asList();

        if (volumePoseTransl == nullptr || volumePoseTransl->size() != 3)
        {
            yCError(KINFU) << "Parameter volumePoseTransl must be a list of 3 floats";
            return nullptr;
        }

        auto transl = cv::Vec3f(volumePoseTransl->get(0).asFloat32(), volumePoseTransl->get(1).asFloat32(), volumePoseTransl->get(2).asFloat32());
        params->volumePose.translation(transl);
        yCInfo(KINFU) << "volumePoseTransl:" << volumePoseTransl->toString();
    }
    else
    {
        const auto & transl = params->volumePose.translation();
        yCInfo(KINFU) << "volumePoseTransl (DEFAULT):" << transl[0] << transl[1] << transl[2];
    }

    if (config.check("volumeType", "type of voxel volume (tsdf, hashtsdf)"))
    {
        std::string volumeType = config.find("volumeType").asString();

        if (stringToCvVolume.find(volumeType) == stringToCvVolume.end())
        {
            yCError(KINFU) << "Unsupported volume type" << volumeType;
            return nullptr;
        }

        params->volumeType = stringToCvVolume[volumeType];
        yCInfo(KINFU) << "volumeType:" << volumeType;
    }
    else
    {
        auto res = std::find_if(stringToCvVolume.begin(), stringToCvVolume.end(), [&params](const auto & el) { return el.second == params->volumeType; });
        yCInfo(KINFU) << "volumeType (DEFAULT):" << res->first;
    }

    updateParam(*params, &Params::voxelSize, config, "voxelSize", "size of voxel in meters");

    return std::make_unique<KinectFusionImpl<cv::colored_kinfu::ColoredKinFu>>(cv::colored_kinfu::ColoredKinFu::create(params));
}

} // namespace roboticslab
