#include <yarp/os/Network.h>

#include "PointAtObjectServer.hpp"

int main(int argc, char *argv[])
{
    printf("Run \"pointAtObjectServer --help\" for options.\n");
    printf("pointAtObjectServer checking for yarp network... ");
    fflush(stdout);
    yarp::os::Network yarp;
    if ( ! yarp.checkNetwork() )
    {
        printf("[fail]\npointAtObjectServer found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else printf("[ok]\n");

    roboticslab::PointAtObjectServer pointAtObjectServer;
    pointAtObjectServer.init();  // init() is a blocking function
    pointAtObjectServer.close();

    return 0;
}

