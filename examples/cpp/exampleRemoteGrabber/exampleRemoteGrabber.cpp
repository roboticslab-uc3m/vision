// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleRemoteGrabber exampleRemoteGrabber
 * @brief This example connects to a remote grabber (generally, RGB) device.
 */

#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

    yarp::os::Property options;

    options.put("device","remote_grabber");
    options.put("local","/exampleRemoteGrabber");
    options.put("remote","/grabber");

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        std::printf("Device not available.\n");
        return 1;
    }

    yarp::dev::IFrameGrabberImage *iFrameGrabberImage;

    if (!dd.view(iFrameGrabberImage))
    {
        std::printf("[error] Problems acquiring interface\n");
        return 1;
    }

    std::printf("[success] acquired interface\n");

    // The following delay should avoid bad status
    yarp::os::Time::delay(1);

    std::printf("Width: %d\n", iFrameGrabberImage->width());
    std::printf("Height: %d\n", iFrameGrabberImage->height());

    dd.close();

    return 0;
}
