namespace yarp roboticslab

struct yarp_sig_Matrix {
} (
    yarp.name = "yarp::sig::Matrix"
    yarp.includefile = "yarp/sig/Matrix.h"
)

struct yarp_sig_PointCloudXYZ {
} (
    yarp.name = "yarp::sig::PointCloudXYZ"
    yarp.includefile = "yarp/sig/PointCloud.h"
)

struct yarp_sig_PointCloudXYZNormalRGBA {
} (
    yarp.name = "yarp::sig::PointCloudXYZNormalRGBA"
    yarp.includefile = "yarp/sig/PointCloud.h"
)

struct return_pose {
    1: bool ret = false;
    2: yarp_sig_Matrix pose;
}

struct return_points {
    1: bool ret = false;
    2: yarp_sig_PointCloudXYZ points;
}

struct return_points_with_normals {
    1: bool ret = false;
    2: yarp_sig_PointCloudXYZNormalRGBA pointsWithNormals;
}

service SceneReconstructionIDL
{
    /**
     * pause the scene reconstruction process
     */
    void pause();

    /**
     * start/resume the scene reconstruction process
     */
    void resume();

    /**
     * get current camera pose
     * @return struct {bool ret, yarp_sig_Matrix pose}
     */
    return_pose getPose();

    /**
     * get the resulting point cloud
     * @return struct {bool ret, yarp_sig_PointCloudXYZ points}
     */
    return_points getPoints();

    /**
     * get the resulting point cloud with normals and color data (if available)
     * @return struct {bool ret, yarp_sig_PointCloudXYZNormalRGBA pointsWithNormals}
     */
    return_points_with_normals getPointsWithNormals();
}
