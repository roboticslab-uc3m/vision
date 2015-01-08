// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LAUNCH_HEAD__
#define __LAUNCH_HEAD__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include <string>
#include <stdlib.h>

#include "ColorDebug.hpp"

#define DEFAULT_KINECT "on"

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 * @ingroup LaunchHead
 *
 * The LaunchHead class.
 * 
 */
class LaunchHead : public RFModule {

    public:
        LaunchHead();
        bool configure(ResourceFinder &rf);

protected:

    virtual double getPeriod() {return 3.0;}
    virtual bool updateModule();
    virtual bool close();

//        virtual bool interruptModule();
//        virtual int period;
    yarp::dev::PolyDriver motorDevice;
    yarp::dev::PolyDriver kinectDevice;

    std::string kinect;

};

}  // namespace teo

#endif  // __LAUNCH_HEAD__

