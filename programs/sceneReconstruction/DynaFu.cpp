// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinectFusionImpl.hpp"

#include <map>

#include <yarp/os/LogStream.h>

#include <opencv2/core/version.hpp>
#include <opencv2/rgbd/dynafu.hpp>

#include "LogComponent.hpp"

#if CV_VERSION_MAJOR > 4 || CV_VERSION_MINOR >= 5
namespace
{
    std::map<std::string, cv::kinfu::VolumeType> stringToCvVolume {
        {"tsdf", cv::kinfu::VolumeType::TSDF},
        {"hashtsdf", cv::kinfu::VolumeType::HASHTSDF},
#if HAVE_COLORED_KINFU
        {"coloredtsdf", cv::kinfu::VolumeType::COLOREDTSDF}
#endif
    };
}
#endif

namespace roboticslab
{

std::unique_ptr<KinectFusion> makeDynaFu(const yarp::os::Searchable & config, const yarp::sig::IntrinsicParams & intrinsic, int width, int height)
{
    using Params = cv::dynafu::Params;

    auto params = Params::defaultParams();

    yCInfo(KINFU) << "--- CAMERA PARAMETERS ---";

    params->frameSize = cv::Size(width, height);
    yCInfo(KINFU) << "width:" << width;
    yCInfo(KINFU) << "height:" << height;

    params->intr = cv::Matx33f(intrinsic.focalLengthX,                      0, intrinsic.principalPointX,
                                                    0, intrinsic.focalLengthY, intrinsic.principalPointY,
                                                    0,                      0,                         1);

    yCInfo(KINFU) << "focal length (X):" << intrinsic.focalLengthX;
    yCInfo(KINFU) << "focal length (Y):" << intrinsic.focalLengthY;
    yCInfo(KINFU) << "principal point (X):" << intrinsic.principalPointX;
    yCInfo(KINFU) << "principal point (Y):" << intrinsic.principalPointY;

    yCInfo(KINFU) << "--- ALGORITHM PARAMETERS ---";

    yCInfo(KINFU) << "algorithm: dynafu";

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

#if CV_VERSION_MAJOR > 4 || CV_VERSION_MINOR >= 5
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
#endif

    updateParam(*params, &Params::voxelSize, config, "voxelSize", "size of voxel in meters");

    return std::make_unique<KinectFusionImpl<cv::dynafu::DynaFu>>(cv::dynafu::DynaFu::create(params));
}

} // namespace roboticslab
