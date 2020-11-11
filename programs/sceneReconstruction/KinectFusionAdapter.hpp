// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KINECT_FUSION_ADAPTER_HPP__
#define __KINECT_FUSION_ADAPTER_HPP__

#include <memory>

#include <yarp/os/Searchable.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/IntrinsicParams.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/PointCloud.h>

namespace roboticslab
{

class KinectFusionAdapter
{
public:
    virtual ~KinectFusionAdapter() {}

    virtual void getCloud(yarp::sig::PointCloudXYZNormal & cloudWithNormals) = 0;

    virtual void getPoints(yarp::sig::PointCloudXYZ & cloud) = 0;

    virtual void getNormals(yarp::sig::PointCloudNormal & normals) = 0;

    virtual void getPose(yarp::sig::Matrix & pose) = 0;

    virtual bool update(const yarp::sig::ImageOf<yarp::sig::PixelFloat> & depthFrame) = 0;

    virtual void reset() = 0;

    virtual void render(yarp::sig::ImageOf<yarp::sig::PixelRgb> & image) = 0;
};

std::unique_ptr<KinectFusionAdapter> makeKinFu(const yarp::os::Searchable & config, const yarp::sig::IntrinsicParams & intrinsic, int width, int height);

} // namespace roboticslab

#endif // __KINECT_FUSION_ADAPTER_HPP__
