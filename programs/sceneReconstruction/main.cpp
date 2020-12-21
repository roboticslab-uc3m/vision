// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_programs
 *
 * @defgroup sceneReconstruction sceneReconstruction
 *
 * @brief Creates an instance of roboticslab::SceneReconstruction.
 *
 * @section sceneReconstructionOptions Configuration parameters
 *
 * | PROPERTY     | DESCRIPTION                          | DEFAULT                 |
 * |--------------|--------------------------------------|-------------------------|
 * | from         | file.ini                             | sceneReconstruction.ini |
 * | context      | context name                         | sceneReconstruction     |
 * | period       | update period (ms)                   | 20                      |
 * | prefix       | prefix for local port names          | /sceneReconstruction    |
 * | remote       | remote port to connect to (optional) |                         |
 * | carrier      | carrier name for remote depth stream |                         |
 *
 * @section  sceneReconstructionPorts Exposed ports
 *
 * | PORT               | CONTENT                                                              |
 * |--------------------|----------------------------------------------------------------------|
 * | /<prefix>/rpc:s    | RPC server for handling remote commands                              |
 * | /<prefix>/render:o | PixelMono Phong-rendered representation of the reconstructed surface |
 *
 * @section sceneReconstructionKinectFusion Kinect Fusion options
 *
 * | PROPERTY              | DESCRIPTION                                  | DEFAULT                               |
 * |-----------------------|----------------------------------------------|---------------------------------------|
 * | algorithm             | Kinect Fusion algorithm ("kinfu", "dynafu")  | kinfu                                 |
 * | bilateralKernelSize   | kernel size in pixels for bilateral smooth   | 7                                     |
 * | bilateralSigmaDepth   | depth sigma in meters for bilateral smooth   | 0.04                                  |
 * | bilateralSigmaSpatial | spatial sigma in pixels for bilateral smooth | 4.5                                   |
 * | depthFactor           | pre-scale per 1 meter for input values       | 5000                                  |
 * | icpAngleThresh        | angle threshold in radians for ICP           | 0.523599                              |
 * | icpDistThresh         | distance threshold in meters for ICP         | 0.1                                   |
 * | icpIterations         | iterations per each ICP level                | (10 5 4)                              |
 * | lightPose             | light pose for rendering in meters           | (0.0 0.0 0.0)                         |
 * | pyramidLevels         | number of pyramid levels for ICP             | 3                                     |
 * | raycastStepFactor     | a lenght in voxel sizes for one raycast step | 0.25                                  |
 * | truncateThreshold     | threshold for depth truncation in meters     | 0.0                                   |
 * | tsdfMaxWeight         | max number of frames per voxel               | 64                                    |
 * | tsdfMinCameraMovement | minimal camera movement in meters            | 0.0                                   |
 * | tsdfTruncDist         | distance to truncate in meters               | 0.0410156                             |
 * | volumeDims            | resolution of voxel space                    | (512 512 512)                         |
 * | volumePoseRot         | volume pose (rotation matrix) in radians     | (1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0) |
 * | volumePoseTransl      | volume pose (translation vector) in meters   | (-1.5 -1.5 0.5)                       |
 * | volumeType            | type of voxel volume ("tsdf", "hashtsdf")    | tsdf                                  |
 * | voxelSize             | size of voxel in meters                      | 0.00585938                            |
 *
 *  @section sceneReconstructionLicense License
 *
 * @see LICENSE_KinectFusion.md
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "SceneReconstruction.hpp"

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << argv[0] << "found no yarp network (try running \"yarpserver &\"), bye!";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("sceneReconstruction");
    rf.setDefaultConfigFile("sceneReconstruction.ini");
    rf.configure(argc, argv);

    roboticslab::SceneReconstruction mod;
    return mod.runModule(rf);
}
