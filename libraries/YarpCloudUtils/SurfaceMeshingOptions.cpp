// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SurfaceMeshingOptions.hpp"

#include <cctype>
#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>

using namespace roboticslab;

namespace
{
    std::string normalizeKey(const std::string & key, const std::string & prefix)
    {
        if (key.find(prefix, 0) == 0)
        {
            return key;
        }
        else
        {
            auto copy = key;
            copy[0] = ::toupper(copy[0]);
            return prefix + copy;
        }
    }

    yarp::os::Property normalizePrefix(const yarp::os::Property & options, const std::string & prefix)
    {
        yarp::os::Property out;
        yarp::os::Bottle b(options.toString());

        for (auto i = 0; i < b.size(); i++)
        {
            const auto * entry = b.get(i).asList();
            auto key = entry->get(0).asString();
            out.put(normalizeKey(key, prefix), entry->get(1));
        }

        return out;
    }
}

SurfaceMeshingOptions SurfaceMeshingOptions::fromConfigGroups(const yarp::os::Searchable & config)
{
    SurfaceMeshingOptions instance;

    const auto & cropGroup = config.findGroup("CROP_STEP");

    if (!cropGroup.isNull())
    {
        instance.cropOptions.fromString(cropGroup.toString());
    }

    const auto & downsampleGroup = config.findGroup("DOWNSAMPLING_STEP");

    if (!downsampleGroup.isNull())
    {
        instance.downsampleOptions.fromString(downsampleGroup.toString());
    }

    const auto & smoothGroup = config.findGroup("CLOUD_SMOOTHING_STEP");

    if (!smoothGroup.isNull())
    {
        instance.smoothOptions.fromString(smoothGroup.toString());
    }

    const auto & estimateGroup = config.findGroup("NORMAL_ESTIMATION_STEP");

    if (!estimateGroup.isNull())
    {
        instance.estimateOptions.fromString(estimateGroup.toString());
    }

    const auto & surfaceGroup = config.findGroup("RECONSTRUCTION_STEP");

    if (!surfaceGroup.isNull())
    {
        instance.surfaceOptions.fromString(surfaceGroup.toString());
    }
    else
    {
        yWarning() << "no group found for mandatory reconstruction step";
    }

    const auto & processGroup = config.findGroup("MESH_PROCESSING_STEP");

    if (!processGroup.isNull())
    {
        instance.processOptions.fromString(processGroup.toString());
    }

    instance.enableMeshSimplification = config.check("simplifyMesh", yarp::os::Value(false)).asBool();

    return instance;
}

SurfaceMeshingOptions SurfaceMeshingOptions::fromSurfaceConfig(const yarp::os::Property & surfaceOptions)
{
    SurfaceMeshingOptions instance;
    instance.surfaceOptions = surfaceOptions;
    return instance;
}

SurfaceMeshingOptions & SurfaceMeshingOptions::addCropStep(const yarp::os::Property & cropOptions)
{
    if (!cropOptions.toString().empty() && hasCropStep())
    {
        yWarning() << "overriding crop step options";
    }

    this->cropOptions = cropOptions;
    return *this;
}

SurfaceMeshingOptions & SurfaceMeshingOptions::addDownsamplingStep(const yarp::os::Property & downsampleOptions)
{
    if (!downsampleOptions.toString().empty() && hasDownsamplingStep())
    {
        yWarning() << "overriding downsampling step options";
    }

    this->downsampleOptions = downsampleOptions;
    return *this;
}

SurfaceMeshingOptions & SurfaceMeshingOptions::addCloudSmoothingStep(const yarp::os::Property & smoothOptions)
{
    if (!smoothOptions.toString().empty() && hasCloudSmoothingStep())
    {
        yWarning() << "overriding cloud smoothing step options";
    }

    this->smoothOptions = smoothOptions;
    return *this;
}

SurfaceMeshingOptions & SurfaceMeshingOptions::addNormalEstimationStep(const yarp::os::Property & estimateOptions)
{
    if (!estimateOptions.toString().empty() && hasNormalEstimationStep())
    {
        yWarning() << "overriding normal estimation step options";
    }

    this->estimateOptions = estimateOptions;
    return *this;
}

SurfaceMeshingOptions & SurfaceMeshingOptions::addMeshProcessingStep(const yarp::os::Property & processOptions)
{
    if (!processOptions.toString().empty() && hasMeshProcessingStep())
    {
        yWarning() << "overriding mesh processing step options";
    }

    this->processOptions = processOptions;
    return *this;
}

SurfaceMeshingOptions & SurfaceMeshingOptions::addMeshSimplificationStep()
{
    enableMeshSimplification = true;
    return *this;
}

yarp::os::Property SurfaceMeshingOptions::compile() const
{
    yarp::os::Property out;

    if (hasCropStep())
    {
        auto normalized = normalizePrefix(cropOptions, "crop");
        out.fromString(normalized.toString(), false);
    }

    if (hasDownsamplingStep())
    {
        auto normalized = normalizePrefix(downsampleOptions, "downsample");
        out.fromString(normalized.toString(), false);
    }

    if (hasCloudSmoothingStep())
    {
        auto normalized = normalizePrefix(smoothOptions, "smooth");
        out.fromString(normalized.toString(), false);
    }

    if (hasNormalEstimationStep())
    {
        auto normalized = normalizePrefix(estimateOptions, "estimator");
        out.fromString(normalized.toString(), false);
    }

    if (hasReconstructionStep())
    {
        auto normalized = normalizePrefix(surfaceOptions, "surface");
        out.fromString(normalized.toString(), false);
    }

    if (hasMeshProcessingStep())
    {
        auto normalized = normalizePrefix(processOptions, "process");
        out.fromString(normalized.toString(), false);
    }

    out.put("simplifyMesh", yarp::os::Value(enableMeshSimplification));

    return out;
}
