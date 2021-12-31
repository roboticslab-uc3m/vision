/**
 * @ingroup vision_examples
 * @defgroup exampleMeshFromLiveRGBD exampleMeshFromLiveRGBD
 * @brief Transform RGBD frame to cloud/mesh.
 */

#include <utility> // std::move

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/IntrinsicParams.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/PointCloudUtils.h>

#include <YarpCloudUtils.hpp>

constexpr const char * DEFAULT_REMOTE = "/realsense2";
constexpr const char * DEFAULT_PREFIX = "/exampleMeshFromLiveRGBD";
constexpr const char * DEFAULT_COLLECTION = "meshPipeline";

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "No YARP server available";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    yarp::os::Property options(rf.toString().c_str());

#ifdef SAMPLE_CONFIG
    // set sensible defaults, most suitable for organized clouds
    std::string sampleConfigFile = SAMPLE_CONFIG;
    options.fromConfigFile(sampleConfigFile, false);
#endif

    yDebug() << "Config:" << options.toString();

    if (options.check("help"))
    {
        yInfo() << argv[0] << "Commands:";
        yInfo() << "\t--remote" << "\tremote port prefix to connect to, defaults to:" << DEFAULT_REMOTE;
        yInfo() << "\t--prefix" << "\tlocal port prefix, defaults to:" << DEFAULT_PREFIX;
        yInfo() << "\t--cloud " << "\tpath to file with .ply extension to export the point cloud to";
        yInfo() << "\t--mesh  " << "\tpath to file with .ply extension to export the surface mesh to";
        yInfo() << "\t--steps " << "\tsection collection defining the meshing pipeline, defaults to:" << DEFAULT_COLLECTION;
        yInfo() << "\t--binary" << "\texport data in binary format, defaults to: true";
        return 0;
    }

    auto remote = options.check("remote", yarp::os::Value(DEFAULT_REMOTE)).asString();
    auto prefix = options.check("prefix", yarp::os::Value(DEFAULT_PREFIX)).asString();
    auto fileCloud = options.check("cloud", yarp::os::Value("")).asString();
    auto fileMesh = options.check("mesh", yarp::os::Value("")).asString();
    auto collection = options.check("steps", yarp::os::Value(DEFAULT_COLLECTION)).asString();
    auto binary = options.check("binary", yarp::os::Value(true)).asBool();

    yarp::sig::FlexImage colorImage;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;
    yarp::sig::IntrinsicParams colorParams;

    {
        yarp::os::Property sensorOptions {
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
            yError() << "Unable to establish connection with remote RGBD sensor";
            return 1;
        }

        yarp::dev::IRGBDSensor * iRGBDSensor;

        if (!sensorDevice.view(iRGBDSensor))
        {
            yError() << "Unable to view IRGBDSensor interface";
            return 1;
        }

        yarp::os::Property intrinsic;

        if (!iRGBDSensor->getRgbIntrinsicParam(intrinsic))
        {
            yError() << "Unable to retrieve RGB intrinsic parameters";
            return 1;
        }

        colorParams.fromProperty(intrinsic);

        for (auto n = 0;; n++)
        {
            iRGBDSensor->getImages(colorImage, depthImage);

            if (colorImage.getRawImageSize() != 0 && depthImage.getRawImageSize() != 0)
            {
                break;
            }
            else if (n == 10)
            {
                yError() << "Unable to acquire RGBD frames";
                return 1;
            }

            yarp::os::SystemClock::delaySystem(0.1);
        }
    }

    yarp::sig::ImageOf<yarp::sig::PixelRgb> temp;
    temp.getPixelCode() == colorImage.getPixelCode() ? temp.move(std::move(colorImage)) : temp.copy(colorImage);
    auto cloud = yarp::sig::utils::depthRgbToPC<yarp::sig::DataXYZRGBA>(depthImage, temp, colorParams);
    yInfo() << "Got cloud of" << cloud.size() << "points, organized as" << cloud.width() << "x" << cloud.height();

    if (!fileCloud.empty())
    {
        if (!roboticslab::YarpCloudUtils::savePLY(fileCloud, cloud, binary))
        {
            yWarning() << "Unable to export cloud to" << fileCloud;
        }
        else
        {
            yInfo() << "Cloud exported to" << fileCloud;
        }
    }

    if (!fileMesh.empty())
    {
        yarp::sig::PointCloudXYZRGBA meshPoints;
        yarp::sig::VectorOf<int> meshIndices;

        auto start = yarp::os::SystemClock::nowSystem();

        if (!roboticslab::YarpCloudUtils::meshFromCloud(cloud, meshPoints, meshIndices, options, collection))
        {
            yWarning() << "Unable to reconstruct surface from cloud";
        }
        else
        {
            auto end = yarp::os::SystemClock::nowSystem();
            auto elapsed = static_cast<int>((end - start) * 1000);

            yInfo() << "Surface reconstructed in" << elapsed << "ms, got mesh of" << meshPoints.size() << "points and"
                    << meshIndices.size() / 3 << "faces";

            if (!roboticslab::YarpCloudUtils::savePLY(fileMesh, meshPoints, meshIndices, binary))
            {
                yWarning() << "Unable to export mesh to" << fileMesh;
            }
            else
            {
                yInfo() << "Mesh exported to" << fileMesh;
            }
        }
    }

    return 0;
}
