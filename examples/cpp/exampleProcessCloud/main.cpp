/**
 * @ingroup vision_examples
 * @defgroup exampleProcessCloud exampleProcessCloud
 * @brief Sample usage of roboticslab::YarpCloudUtils::processCloud.
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>

#include <YarpCloudUtils.hpp>

constexpr const char * DEFAULT_COLLECTION = "cloudPipeline";

int main(int argc, char * argv[])
{
    yarp::os::Property options;
    options.fromCommand(argc, argv);

    if (options.check("help"))
    {
        yInfo() << argv[0] << "commands:";
        yInfo() << "\t--in    " << "\tpath to file with .ply extension to import the point cloud from";
        yInfo() << "\t--out   " << "\tpath to file with .ply extension to export the point cloud to";
        yInfo() << "\t--steps " << "\tsection collection defining the processing pipeline, defaults to:" << DEFAULT_COLLECTION;
        yInfo() << "\t--binary" << "\texport data in binary format, defaults to: true";
        yInfo() << "\t--height" << "\tnumber of rows (for organized clouds)";
        yInfo() << "\t--width " << "\tnumber of columns (for organized clouds)";
        return 0;
    }

    auto fileIn = options.check("in", yarp::os::Value("")).asString();
    auto fileOut = options.check("out", yarp::os::Value("")).asString();
    auto collection = options.check("steps", yarp::os::Value(DEFAULT_COLLECTION)).asString();
    auto binary = options.check("binary", yarp::os::Value(true)).asBool();

    if (fileIn.empty() || fileOut.empty())
    {
        yError() << "either of the --in and --out parameters are missing or empty";
        return 1;
    }

    yarp::sig::PointCloudXYZ cloudIn;

    if (!roboticslab::YarpCloudUtils::loadPLY(fileIn, cloudIn))
    {
        yError() << "unable to import cloud from" << fileIn;
        return 1;
    }

    yInfo() << "got cloud of" << cloudIn.size() << "points";

    if (options.check("height") && options.check("width"))
    {
        auto height = options.find("height").asInt32();
        auto width = options.find("width").asInt32();

        if (height * width != cloudIn.size())
        {
            yWarning() << "organized cloud dimensions do not match number of points:" << height << "*" << width << "=" << height * width;
        }

        cloudIn.resize(width, height);
    }

    yarp::sig::PointCloudXYZ cloudOut;

    auto start = yarp::os::SystemClock::nowSystem();

    if (!roboticslab::YarpCloudUtils::processCloud(cloudIn, cloudOut, options, collection))
    {
        yError() << "unable to process cloud";
        return 1;
    }

    auto end = yarp::os::SystemClock::nowSystem();
    auto elapsed = static_cast<int>((end - start) * 1000);

    yInfo() << "cloud processed in" << elapsed << "ms, got new cloud of" << cloudOut.size() << "points";

    if (!roboticslab::YarpCloudUtils::savePLY(fileOut, cloudOut, binary))
    {
        yError() << "unable to export mesh to" << fileOut;
        return 1;
    }

    yInfo() << "cloud exported to" << fileOut;

    return 0;
}
