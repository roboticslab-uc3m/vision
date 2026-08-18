// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinectFusionImpl.hpp"

#include <map>

#include <yarp/os/LogStream.h>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
# include <opencv2/ptcloud.hpp>
#endif
#include <opencv2/rgbd/large_kinfu.hpp>

#include "LogComponent.hpp"

namespace
{
#if CV_VERSION_MAJOR >= 5
    std::map<std::string, cv::VolumeType> stringToCvVolume {
        {"tsdf", cv::VolumeType::TSDF},
        {"hashtsdf", cv::VolumeType::HashTSDF},
        {"coloredtsdf", cv::VolumeType::ColorTSDF}
    };
#else
    std::map<std::string, cv::kinfu::VolumeType> stringToCvVolume {
        {"tsdf", cv::kinfu::VolumeType::TSDF},
        {"hashtsdf", cv::kinfu::VolumeType::HASHTSDF},
        {"coloredtsdf", cv::kinfu::VolumeType::COLOREDTSDF}
    };
#endif
}

namespace roboticslab
{

std::unique_ptr<KinectFusion> makeKinFuLargeScale(const yarp::os::Searchable & config, const yarp::sig::IntrinsicParams & intrinsic, int width, int height)
{
    using Params = cv::large_kinfu::Params;
#if CV_VERSION_MAJOR >= 5
    using VolParams = cv::large_kinfu::VolumeParams;
#else
    using VolParams = cv::kinfu::VolumeParams;
#endif

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

    yCInfo(KINFU) << "algorithm: kinfu_ls";

    updateParam(*params, &Params::bilateral_kernel_size, config, "bilateralKernelSize", "kernel size in pixels for bilateral smooth");
    updateParam(*params, &Params::bilateral_sigma_depth, config, "bilateralSigmaDepth", "depth sigma in meters for bilateral smooth");
    updateParam(*params, &Params::bilateral_sigma_spatial, config, "bilateralSigmaSpatial", "spatial sigma in pixels for bilateral smooth");
    updateParam(*params, &Params::depthFactor, config, "depthFactor", "pre-scale per 1 meter for input values");
    updateParam(params->volumeParams, &VolParams::depthTruncThreshold, config, "depthTruncThreshold", "threshold for depth truncation in meters");
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
    updateParam(params->volumeParams, &VolParams::raycastStepFactor, config, "raycastStepFactor", "a length in voxel sizes for one raycast step");
    updateParam(*params, &Params::truncateThreshold, config, "truncateThreshold", "threshold for depth truncation in meters");
    updateParam(params->volumeParams, &VolParams::maxWeight, config, "tsdfMaxWeight", "max number of frames per voxel");
    updateParam(*params, &Params::tsdf_min_camera_movement, config, "tsdfMinCameraMovement", "minimal camera movement in meters");
    updateParam(params->volumeParams, &VolParams::tsdfTruncDist, config, "tsdfTruncDist", "distance to truncate in meters");

    if (config.check("volumeDims", "resolution of voxel space"))
    {
        yarp::os::Bottle * volumeDims = config.find("volumeDims").asList();

        if (volumeDims == nullptr || volumeDims->size() != 3)
        {
            yCError(KINFU) << "Parameter volumeDims must be a list of 3 integers";
            return nullptr;
        }

#if CV_VERSION_MAJOR >= 5
        params->volumeParams.resolutionX = volumeDims->get(0).asInt32();
        params->volumeParams.resolutionY = volumeDims->get(1).asInt32();
        params->volumeParams.resolutionZ = volumeDims->get(2).asInt32();
#else
        params->volumeParams.resolution = cv::Vec3i(volumeDims->get(0).asInt32(), volumeDims->get(1).asInt32(), volumeDims->get(2).asInt32());
#endif
        yCInfo(KINFU) << "volumeDims:" << volumeDims->toString();
    }
    else
    {
#if CV_VERSION_MAJOR >= 5
        yCInfo(KINFU) << "volumeDims (DEFAULT):" << params->volumeParams.resolutionX
                                                 << params->volumeParams.resolutionY
                                                 << params->volumeParams.resolutionZ;
#else
        const auto & cvDims = params->volumeParams.resolution;
        yCInfo(KINFU) << "volumeDims (DEFAULT):" << cvDims[0] << cvDims[1] << cvDims[2];
#endif
    }

    updateParam(params->volumeParams, &VolParams::unitResolution, config, "unitResolution", "resolution of volumeUnit in voxel space");

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

#if CV_VERSION_MAJOR >= 5
        params->volumeParams.pose = cv::Affine3f().rotate(rot).matrix;
#else
        params->volumeParams.pose.rotation(rot);
#endif
        yCInfo(KINFU) << "volumePoseRot:" << volumePoseRot->toString();
    }
    else
    {
#if CV_VERSION_MAJOR >= 5
        const auto rot = params->volumeParams.pose.get_minor<3, 3>(0, 0);
#else
        const auto & rot = params->volumeParams.pose.rotation();
#endif
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

#if CV_VERSION_MAJOR >= 5
        params->volumeParams.pose(0, 3) = volumePoseTransl->get(0).asFloat32();
        params->volumeParams.pose(1, 3) = volumePoseTransl->get(1).asFloat32();
        params->volumeParams.pose(2, 3) = volumePoseTransl->get(2).asFloat32();
#else
        auto transl = cv::Vec3f(volumePoseTransl->get(0).asFloat32(), volumePoseTransl->get(1).asFloat32(), volumePoseTransl->get(2).asFloat32());
        params->volumeParams.pose.translation(transl);
#endif
        yCInfo(KINFU) << "volumePoseTransl:" << volumePoseTransl->toString();
    }
    else
    {
#if CV_VERSION_MAJOR >= 5
        const auto transl = params->volumeParams.pose.get_minor<3, 1>(0, 3);
#else
        const auto & transl = params->volumeParams.pose.translation();
#endif
        yCInfo(KINFU) << "volumePoseTransl (DEFAULT):" << transl(0) << transl(1) << transl(2);
    }

    if (config.check("volumeType", "type of voxel volume (tsdf, hashtsdf)"))
    {
        std::string volumeType = config.find("volumeType").asString();

        if (stringToCvVolume.find(volumeType) == stringToCvVolume.end())
        {
            yCError(KINFU) << "Unsupported volume type" << volumeType;
            return nullptr;
        }

#if CV_VERSION_MAJOR >= 5
        params->volumeParams.kind = stringToCvVolume[volumeType];
#else
        params->volumeParams.type = stringToCvVolume[volumeType];
#endif
        yCInfo(KINFU) << "volumeType:" << volumeType;
    }
    else
    {
#if CV_VERSION_MAJOR >= 5
        auto res = std::find_if(stringToCvVolume.begin(), stringToCvVolume.end(), [&params](const auto & el) { return el.second == params->volumeParams.kind; });
#else
        auto res = std::find_if(stringToCvVolume.begin(), stringToCvVolume.end(), [&params](const auto & el) { return el.second == params->volumeParams.type; });
#endif
        yCInfo(KINFU) << "volumeType (DEFAULT):" << res->first;
    }

    updateParam(params->volumeParams, &VolParams::voxelSize, config, "voxelSize", "size of voxel in meters");

    return std::make_unique<KinectFusionImpl<cv::large_kinfu::LargeKinfu>>(cv::large_kinfu::LargeKinfu::create(params));
}

} // namespace roboticslab
