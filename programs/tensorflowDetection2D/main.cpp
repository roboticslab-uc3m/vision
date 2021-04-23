/*
 * ************************************************************
 *      Program: Tensorflow Detection 2D Module
 *      Type: main.cpp
 *      Author: David Velasco Garcia @davidvelascogarcia
 * ************************************************************
 */

/*
  *
  * | INPUT PORT                      | CONTENT                                                 |
  * |---------------------------------|---------------------------------------------------------|
  * | /tensorflowDetection2D/img:i    | Input image                                             |
  * | /tensorflowDetection2D/shape   |  Pre-configure tensorflow width and height image         |
  *
  *
  * | OUTPUT PORT                     | CONTENT                                                 |
  * |---------------------------------|---------------------------------------------------------|
  * | /tensorflowDetection2D/img:o    | Output image with object detection                      |
  * | /tensorflowDetection2D/results  | Output result, object, score and number                 |
  *
  */

/**
 * @ingroup vision_programs
 * @defgroup tensorflowDetection2D
 * @brief Creates an instance of roboticslab::TensorflowDetection2D.
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include "TensorflowDetection2D.hpp"

int main(int argc, char** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("tensorflowDetection");
    rf.setDefaultConfigFile("tensorflowDetection2D.ini");
    rf.configure(argc, argv);

    roboticslab::TensorflowDetection2D mod;

    if (rf.check("help"))
    {
        return mod.runModule(rf);
    }

    yInfo("Run \"%s --help\" for options", argv[0]);
    yInfo("%s checking for yarp network...", argv[0]);

    std::fflush(stdout);

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << argv[0] << "found no yarp network (try running \"yarpserver &\"), bye!";
        return 1;
    }

    return mod.runModule(rf);
}
