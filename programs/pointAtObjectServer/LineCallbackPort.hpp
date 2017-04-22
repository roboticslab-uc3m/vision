// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINE_CALLBACK_PORT_HPP__
#define __LINE_CALLBACK_PORT_HPP__

#include <yarp/os/BufferedPort.h>

#include "SharedArea.hpp"

using namespace yarp::os;

namespace pointatobject {

class LineCallbackPort : public BufferedPort<Bottle> {
    protected:
        void onRead(Bottle& b);
        SharedArea* sharedArea;

    public:
        void setSharedArea(SharedArea* _sharedArea);
};

}  // pointatobject

#endif  // __LINE_CALLBACK_PORT_HPP__

