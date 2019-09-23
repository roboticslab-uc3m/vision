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


#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <ColorDebug.h>
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

    CD_INFO("Run \"%s --help\" for options.\n", argv[0]);
    CD_INFO("%s checking for yarp network... ", argv[0]);

    std::fflush(stdout);

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("%s found no yarp network (try running \"yarpserver &\"), bye!\n", argv[0]);
        return 1;
    }
    else
    {
        CD_SUCCESS_NO_HEADER("[ok]\n");
    }

    return mod.runModule(rf);
}
