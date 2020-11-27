#include <chrono>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <YarpCloudUtils.hpp>

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    if (rf.check("help"))
    {
        yInfo() << argv[0] << "commands:";
        yInfo() << "\t--cloud" << "\tpath to file with .ply extension to export the point cloud to";
        yInfo() << "\t--mesh " << "\tpath to file with .ply extension to export the surface mesh to";
        yInfo() << "\t--binary" << "\texport data in binary format (default: true)";
        yInfo() << "additional parameters are used to configure the surface reconstruction method, if requested";
        return 0;
    }

    auto fileCloud = rf.check("cloud", yarp::os::Value("")).asString();
    auto fileMesh = rf.check("mesh", yarp::os::Value("")).asString();
    auto binary = rf.check("binary", yarp::os::Value(true)).asBool();

    if (fileCloud.empty() || fileMesh.empty())
    {
        yError() << "missing or empty --cloud and/or --mesh parameters";
        return 1;
    }

    yarp::sig::PointCloudXYZ cloud;

    if (!roboticslab::YarpCloudUtils::loadPLY(fileCloud, cloud))
    {
        yError() << "unable to import cloud from" << fileCloud;
        return 1;
    }

    yInfo() << "got cloud of" << cloud.size() << "points";

    yarp::sig::PointCloudXYZ meshPoints;
    yarp::sig::VectorOf<int> meshIndices;

    auto start = std::chrono::system_clock::now();

    if (!roboticslab::YarpCloudUtils::meshFromCloud(cloud, meshPoints, meshIndices, rf))
    {
        yError() << "unable to reconstruct surface from cloud";
        return 1;
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    yInfo() << "surface reconstructed in" << elapsed.count() << "ms, got mesh of" << meshPoints.size() << "points and"
            << meshIndices.size() << "indices";

    if (!roboticslab::YarpCloudUtils::savePLY(fileMesh, meshPoints, meshIndices, binary))
    {
        yError() << "unable to export mesh to" << fileMesh;
        return 1;
    }

    yInfo() << "mesh exported to" << fileMesh;

    return 0;
}
