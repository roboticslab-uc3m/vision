// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CALLBACK_PORT_HPP__
#define __CALLBACK_PORT_HPP__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;

class CallbackPort : public BufferedPort<Bottle> {
    private:
        Port* outPort;
        BufferedPort<ImageOf<PixelFloat> >* depthPort;
        virtual void onRead(Bottle& b);
        double fx, fy, cx, cy;

    public:
        void setParams(double _fx, double _fy, double _cx, double _cy);
        void setDepthPort(BufferedPort<ImageOf<PixelFloat> >* _depthPort);
        void setOutPort(Port* _outPort);
};

#endif  // __CALLBACK_PORT_HPP__

