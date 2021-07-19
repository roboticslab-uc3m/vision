/**
 * @ingroup vision_examples
 * @defgroup exampleSceneReconstructionClient exampleSceneReconstructionClient
 * @brief Sample usage of @ref sceneReconstruction.
 */

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>
#include <yarp/sig/PointCloud.h>

#include <YarpCloudUtils.hpp>

constexpr auto DEFAULT_REMOTE = "/sceneReconstruction";
constexpr auto DEFAULT_PREFIX = "/exampleSceneReconstructionClient";
constexpr auto DEFAULT_COLLECTION = "meshPipeline";

#if YARP_VERSION_MINOR >= 5
constexpr auto VOCAB_GET_POINTS = yarp::os::createVocab32('g','p','c');
#else
constexpr auto VOCAB_GET_POINTS = yarp::os::createVocab('g','p','c');
#endif

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "No YARP server available";
        return 1;
    }

    yarp::os::ResourceFinder options;
    options.configure(argc, argv);

    yDebug() << "Config:" << options.toString();

    if (options.check("help"))
    {
        yInfo() << argv[0] << "Commands:";
        yInfo() << "\t--remote" << "\tremote port to connect to, defaults to:" << DEFAULT_REMOTE;
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

    yarp::os::RpcClient rpc;

    if (!rpc.open(prefix + "/rpc:c") || !yarp::os::Network::connect(rpc.getName(), remote + "/rpc:s"))
    {
        yError() << "Unable to establish connection to remote server";
        return 1;
    }

    yarp::os::Bottle cmd {yarp::os::Value(VOCAB_GET_POINTS, true)};
    yarp::sig::PointCloudXYZ cloud;

    if (!rpc.write(cmd, cloud))
    {
        yError() << "Unable to send remote command";
        return 1;
    }

    yInfo() << "Got cloud of" << cloud.size() << "points";

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
        yarp::sig::PointCloudXYZ meshPoints;
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
