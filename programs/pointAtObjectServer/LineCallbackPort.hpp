// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINE_CALLBACK_PORT_HPP__
#define __LINE_CALLBACK_PORT_HPP__

#include <yarp/os/BufferedPort.h>

#include "SharedArea.hpp"

namespace teo {

class LineCallbackPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
    protected:
        void onRead(yarp::os::Bottle& b);
        SharedArea* sharedArea;

    public:
        void setSharedArea(SharedArea* _sharedArea);
};

}  // teo

#endif  // __LINE_CALLBACK_PORT_HPP__

