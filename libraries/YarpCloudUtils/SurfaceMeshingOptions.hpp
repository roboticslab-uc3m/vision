// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SURFACE_MESHING_OPTIONS_HPP__
#define __SURFACE_MESHING_OPTIONS_HPP__

#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>

namespace roboticslab
{

class SurfaceMeshingOptions final
{
public:
    static SurfaceMeshingOptions fromConfigGroups(const yarp::os::Searchable & config);
    static SurfaceMeshingOptions fromSurfaceConfig(const yarp::os::Property & surfaceOptions);

    SurfaceMeshingOptions & addCropStep(const yarp::os::Property & cropOptions);
    SurfaceMeshingOptions & addDownsamplingStep(const yarp::os::Property & downsampleOptions);
    SurfaceMeshingOptions & addCloudSmoothingStep(const yarp::os::Property & smoothOptions);
    SurfaceMeshingOptions & addNormalEstimationStep(const yarp::os::Property & estimateOptions);
    SurfaceMeshingOptions & addMeshProcessingStep(const yarp::os::Property & processOptions);
    SurfaceMeshingOptions & addMeshSimplificationStep();

    bool hasCropStep() const
    { return !cropOptions.toString().empty(); }

    bool hasDownsamplingStep() const
    { return !downsampleOptions.toString().empty(); }

    bool hasCloudSmoothingStep() const
    { return !smoothOptions.toString().empty(); }

    bool hasNormalEstimationStep() const
    { return !estimateOptions.toString().empty(); }

    bool hasReconstructionStep() const
    { return !surfaceOptions.toString().empty(); }

    bool hasMeshProcessingStep() const
    { return !processOptions.toString().empty(); }

    bool hasMeshSimplificationStep() const
    { return enableMeshSimplification; }

    yarp::os::Property compile() const;

private:
    SurfaceMeshingOptions() : enableMeshSimplification(false) {}

    yarp::os::Property cropOptions;
    yarp::os::Property downsampleOptions;
    yarp::os::Property smoothOptions;
    yarp::os::Property estimateOptions;
    yarp::os::Property surfaceOptions;
    yarp::os::Property processOptions;

    bool enableMeshSimplification;
};

} // namespace roboticslab

#endif // __SURFACE_MESHING_OPTIONS_HPP__
