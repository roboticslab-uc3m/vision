#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>
#include <yarp/sig/PointCloud.h>

#include <YarpCloudUtils.hpp>

constexpr const char * DEFAULT_REMOTE = "/sceneReconstruction";
constexpr const char * DEFAULT_PREFIX = "/exampleSceneReconstructionClient";

constexpr auto VOCAB_GET_POINTS = yarp::os::createVocab('g','p','c');
constexpr auto VOCAB_GET_POINTS_AND_NORMALS = yarp::os::createVocab('g','p','c','n');

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
        yInfo() << "\t--remote" << "\tremote port to connect to, defaults to" << DEFAULT_REMOTE;
        yInfo() << "\t--prefix" << "\tlocal port prefix, defaults to" << DEFAULT_PREFIX;
        yInfo() << "\t--export" << "\tpath to file with .ply extension to export the point cloud to";
        return 0;
    }

    if (!rf.check("export"))
    {
        yError() << "missing mandatory --export parameter";
        return 1;
    }

    std::string remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE)).asString();
    std::string prefix = rf.check("prefix", yarp::os::Value(DEFAULT_PREFIX)).asString();
    std::string file = rf.find("export").asString();

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
        yError() << "unable to send first command";
        return 1;
    }

    yInfo() << "got cloud of" << cloud.size() << "points";

    if (!roboticslab::YarpCloudUtils::savePLY(file, cloud, false))
    {
        yError() << "unable to save file to" << file;
        return 1;
    }
    else
    {
        yInfo() << "successfully saved cloud to" << file;
        return 0;
    }
}
