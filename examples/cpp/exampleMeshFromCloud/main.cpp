/**
 * @ingroup vision_examples
 * @defgroup exampleMeshFromCloud exampleMeshFromCloud
 * @brief Sample usage of roboticslab::YarpCloudUtils::meshFromCloud.
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/SystemClock.h>

#include <YarpCloudUtils.hpp>

constexpr const char * DEFAULT_COLLECTION = "meshPipeline";

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder options;
    options.configure(argc, argv);

    yDebug() << "config:" << options.toString();

    if (options.check("help"))
    {
        yInfo() << argv[0] << "commands:";
        yInfo() << "\t--cloud " << "\tpath to file with .ply extension to import the point cloud from";
        yInfo() << "\t--mesh  " << "\tpath to file with .ply extension to export the surface mesh to";
        yInfo() << "\t--steps " << "\tsection collection defining the meshing pipeline, defaults to:" << DEFAULT_COLLECTION;
        yInfo() << "\t--binary" << "\texport data in binary format, defaults to: true";
        yInfo() << "\t--height" << "\tnumber of rows (for organized clouds)";
        yInfo() << "\t--width " << "\tnumber of columns (for organized clouds)";
        return 0;
    }

    auto fileCloud = options.check("cloud", yarp::os::Value("")).asString();
    auto fileMesh = options.check("mesh", yarp::os::Value("")).asString();
    auto collection = options.check("steps", yarp::os::Value(DEFAULT_COLLECTION)).asString();
    auto binary = options.check("binary", yarp::os::Value(true)).asBool();

    if (fileCloud.empty() || fileMesh.empty())
    {
        yError() << "either of the --cloud and --mesh parameters are missing or empty";
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

    if (!roboticslab::YarpCloudUtils::meshFromCloud(cloud, meshPoints, meshIndices, options, collection))
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