// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup vision_examples
 * @defgroup exampleRemoteGrabber exampleRemoteGrabber
 * @brief This example connects to a remote grabber (generally, RGB) device.
 */

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>

#if YARP_VERSION_MINOR >= 5
# include <yarp/dev/IFrameGrabberImage.h>
# include <yarp/dev/IFrameGrabberControls.h>
#else
# include <yarp/dev/FrameGrabberInterfaces.h>
#endif

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options {
        {"device", yarp::os::Value("remote_grabber")},
        {"local", yarp::os::Value("/exampleRemoteGrabber")},
        {"remote", yarp::os::Value("/grabber")},
    };

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        yError() << "Device not available";
        return 1;
    }

    yarp::dev::IFrameGrabberImage *iFrameGrabberImage;
    yarp::dev::IFrameGrabberControls *iFrameGrabberControls;

    if (!dd.view(iFrameGrabberImage))
    {
        yError() << "Problems acquiring image interface";
        return 1;
    }

    if (!dd.view(iFrameGrabberControls))
    {
        yError() << "Problems acquiring controls interface";
        return 1;
    }

    bool has;

    if (iFrameGrabberControls->hasFeature(YARP_FEATURE_ZOOM, &has))
    {
        if (has)
        {
            double val;
            iFrameGrabberControls->getFeature(YARP_FEATURE_ZOOM, &val);
            yInfo() << "Zoom feature:" << val;
        }
        else
            yInfo() << "Zoom feature: not supported";
    }
    else
        yWarning() << "iFrameGrabberControls->hasFeature() failed";

    // The following delay should avoid bad status
    yarp::os::Time::delay(1);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> image;

    if (!iFrameGrabberImage->getImage(image))
    {
        yError() << "Problems getting image";
        return 1;
    }

    yInfo() << "Width:" << iFrameGrabberImage->width();
    yInfo() << "Height:" << iFrameGrabberImage->height();

    dd.close();

    return 0;
}
