#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>
#include <yarp/sig/PointCloud.h>

#include <YarpCloudUtils.hpp>

constexpr const char * DEFAULT_REMOTE = "/sceneReconstruction";
constexpr const char * DEFAULT_PREFIX = "/exampleSceneReconstructionClient";

constexpr auto VOCAB_GET_POINTS = yarp::os::createVocab('g','p','c');

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "no YARP server available";
        return 1;
    }

    yarp::os::Property options;
    options.fromCommand(argc, argv);

    if (options.check("help"))
    {
        yInfo() << argv[0] << "commands:";
        yInfo() << "\t--remote" << "\tremote port to connect to, defaults to" << DEFAULT_REMOTE;
        yInfo() << "\t--prefix" << "\tlocal port prefix, defaults to" << DEFAULT_PREFIX;
        yInfo() << "\t--cloud" << "\tpath to file with .ply extension to export the point cloud to";
        yInfo() << "\t--mesh " << "\tpath to file with .ply extension to export the surface mesh to";
        yInfo() << "\t--binary" << "\texport data in binary format (default: true)";
        yInfo() << "additional parameters are used to configure the surface reconstruction method, if requested";
        return 0;
    }

    auto remote = options.check("remote", yarp::os::Value(DEFAULT_REMOTE)).asString();
    auto prefix = options.check("prefix", yarp::os::Value(DEFAULT_PREFIX)).asString();
    auto fileCloud = options.check("cloud", yarp::os::Value("")).asString();
    auto fileMesh = options.check("mesh", yarp::os::Value("")).asString();
    auto binary = options.check("binary", yarp::os::Value(true)).asBool();

    yarp::os::RpcClient rpc;

    if (!rpc.open(prefix + "/rpc:c") || !yarp::os::Network::connect(rpc.getName(), remote + "/rpc:s"))
    {
        yError() << "unable to establish connection to remote server";
        return 1;
    }

    yarp::os::Bottle cmd {yarp::os::Value(VOCAB_GET_POINTS, true)};
    yarp::sig::PointCloudXYZ cloud;

    if (!rpc.write(cmd, cloud))
    {
        yError() << "unable to send remote command";
        return 1;
    }

    yInfo() << "got cloud of" << cloud.size() << "points";

    if (!fileCloud.empty())
    {
        if (!roboticslab::YarpCloudUtils::savePLY(fileCloud, cloud, binary))
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

        auto start = yarp::os::SystemClock::nowSystem();

        if (!roboticslab::YarpCloudUtils::meshFromCloud(cloud, meshPoints, meshIndices, options))
        {
            yWarning() << "unable to reconstruct surface from cloud";
        }
        else
        {
            auto end = yarp::os::SystemClock::nowSystem();
            auto elapsed = static_cast<int>((end - start) * 1000);

            yInfo() << "surface reconstructed in" << elapsed << "ms, got mesh of" << meshPoints.size() << "points and"
                    << meshIndices.size() / 3 << "faces";

            if (!roboticslab::YarpCloudUtils::savePLY(fileMesh, meshPoints, meshIndices, binary))
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
