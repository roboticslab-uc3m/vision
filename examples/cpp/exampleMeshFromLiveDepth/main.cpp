#include <chrono>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/IntrinsicParams.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/PointCloudUtils.h>

#include <YarpCloudUtils.hpp>

constexpr const char * DEFAULT_REMOTE = "/realsense2";
constexpr const char * DEFAULT_PREFIX = "/exampleMeshFromLiveDepth";

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "no YARP server available";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    if (rf.check("help"))
    {
        yInfo() << argv[0] << "commands:";
        yInfo() << "\t--remote" << "\tremote port prefix to connect to, defaults to" << DEFAULT_REMOTE;
        yInfo() << "\t--prefix" << "\tlocal port prefix, defaults to" << DEFAULT_PREFIX;
        yInfo() << "\t--cloud" << "\tpath to file with .ply extension to export the point cloud to";
        yInfo() << "\t--mesh " << "\tpath to file with .ply extension to export the surface mesh to";
        yInfo() << "additional parameters are used to configure the surface reconstruction method, if requested";
        return 0;
    }

    auto remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE)).asString();
    auto prefix = rf.check("prefix", yarp::os::Value(DEFAULT_PREFIX)).asString();
    auto fileCloud = rf.check("cloud", yarp::os::Value("")).asString();
    auto fileMesh = rf.check("mesh", yarp::os::Value("")).asString();

    yarp::sig::IntrinsicParams depthParams;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;

    {
        yarp::os::Property sensorOptions = {
            {"device", yarp::os::Value("RGBDSensorClient")},
            {"localImagePort", yarp::os::Value(prefix + "/client/rgbImage:i")},
            {"localDepthPort", yarp::os::Value(prefix + "/client/depthImage:i")},
            {"localRpcPort", yarp::os::Value(prefix + "/client/rpc:o")},
            {"remoteImagePort", yarp::os::Value(remote + "/rgbImage:o")},
            {"remoteDepthPort", yarp::os::Value(remote + "/depthImage:o")},
            {"remoteRpcPort", yarp::os::Value(remote + "/rpc:i")}
        };

        yarp::dev::PolyDriver sensorDevice;

        if (!sensorDevice.open(sensorOptions))
        {
            yError() << "unable to establish connection with remote RGBD sensor";
            return 1;
        }

        yarp::dev::IRGBDSensor * iRGBDSensor;

        if (!sensorDevice.view(iRGBDSensor))
        {
            yError() << "unable to view IRGBDSensor interface";
            return 1;
        }

        yarp::os::Property intrinsic;

        if (!iRGBDSensor->getDepthIntrinsicParam(intrinsic))
        {
            yError() << "unable to retrieve depth intrinsic parameters";
            return 1;
        }

        depthParams.fromProperty(intrinsic);

        for (auto n = 0;; n++)
        {
            iRGBDSensor->getDepthImage(depthImage);

            if (depthImage.getRawImageSize() != 0)
            {
                break;
            }
            else if (n == 10)
            {
                yError() << "unable to acquire depth frame";
                return 1;
            }

            yarp::os::Time::delay(0.1);
        }
    }

    auto cloud = yarp::sig::utils::depthToPC(depthImage, depthParams);
    yInfo() << "got cloud of" << cloud.size() << "points";

    if (!fileCloud.empty())
    {
        if (!roboticslab::YarpCloudUtils::savePLY(fileCloud, cloud))
        {
            yWarning() << "unable to export cloud to" << fileCloud;
        }
        else
        {
            yInfo() << "cloud exported to" << fileCloud;
        }
    }

    if (!fileMesh.empty())
    {
        yarp::sig::PointCloudXYZ meshPoints;
        yarp::sig::VectorOf<int> meshIndices;
        auto start = std::chrono::system_clock::now();

        if (!roboticslab::YarpCloudUtils::meshFromCloud(cloud, meshPoints, meshIndices, rf))
        {
            yWarning() << "unable to reconstruct surface from cloud";
        }
        else
        {
            auto end = std::chrono::system_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            yInfo() << "surface reconstructed in" << elapsed.count() << "ms, got mesh of" << meshPoints.size() << "points and"
                    << meshIndices.size() << "indices";

            if (!roboticslab::YarpCloudUtils::savePLY(fileMesh, meshPoints, meshIndices))
            {
                yWarning() << "unable to export mesh to" << fileMesh;
            }
            else
            {
                yInfo() << "mesh exported to" << fileMesh;
            }
        }
    }

    return 0;
}
