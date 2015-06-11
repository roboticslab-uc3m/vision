// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PREMULT_PORTS__
#define __PREMULT_PORTS__

//#include <math.h>

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

using namespace yarp::os;
using namespace yarp::math;

/**
 * @ingroup PremultPorts
 *
 * PremultPorts class implements a port with x callbacks.
 */
class PremultPorts : public BufferedPort<Bottle> {
protected:
    /**
    * Implement the actual callback.
    */
    void onRead(Bottle& b);
    Port* outPort;

public:

    PremultPorts() {}
    void setOutPort(Port* _outPort);
};

#endif

