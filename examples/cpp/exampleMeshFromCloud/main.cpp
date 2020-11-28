#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>

#include <YarpCloudUtils.hpp>

int main(int argc, char * argv[])
{
    yarp::os::Property options;
    options.fromCommand(argc, argv);

    if (options.check("help"))
    {
        yInfo() << argv[0] << "commands:";
        yInfo() << "\t--cloud" << "\tpath to file with .ply extension to export the point cloud to";
        yInfo() << "\t--mesh " << "\tpath to file with .ply extension to export the surface mesh to";
        yInfo() << "\t--binary" << "\texport data in binary format (default: true)";
        yInfo() << "\t--height" << "\tnumber of rows (for organized clouds)";
        yInfo() << "\t--width" << "\tnumber of columns (for organized clouds)";
        yInfo() << "additional parameters are used to configure the surface reconstruction method, if requested";
        return 0;
    }

    auto fileCloud = options.check("cloud", yarp::os::Value("")).asString();
    auto fileMesh = options.check("mesh", yarp::os::Value("")).asString();
    auto binary = options.check("binary", yarp::os::Value(true)).asBool();

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

    if (options.check("height") && options.check("width"))
    {
        auto height = options.find("height").asInt32();
        auto width = options.find("width").asInt32();

        if (height * width != cloud.size())
        {
            yWarning() << "organized cloud dimensions do not match number of points:" << height << "*" << width << "=" << height * width;
        }

        cloud.resize(width, height);
    }

    yarp::sig::PointCloudXYZ meshPoints;
    yarp::sig::VectorOf<int> meshIndices;

    auto start = yarp::os::SystemClock::nowSystem();

    if (!roboticslab::YarpCloudUtils::meshFromCloud(cloud, meshPoints, meshIndices, options))
    {
        yError() << "unable to reconstruct surface from cloud";
        return 1;
    }

    auto end = yarp::os::SystemClock::nowSystem();
    auto elapsed = static_cast<int>((end - start) * 1000);

    yInfo() << "surface reconstructed in" << elapsed << "ms, got mesh of" << meshPoints.size() << "points and"
            << meshIndices.size() / 3 << "faces";

    if (!roboticslab::YarpCloudUtils::savePLY(fileMesh, meshPoints, meshIndices, binary))
    {
        yError() << "unable to export mesh to" << fileMesh;
        return 1;
    }

    yInfo() << "mesh exported to" << fileMesh;

    return 0;
}
