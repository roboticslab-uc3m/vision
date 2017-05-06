#ifndef __LINE_CALLBACK_PORT_HPP__
#define __LINE_CALLBACK_PORT_HPP__

#include <yarp/os/BufferedPort.h>

#include "SharedArea.hpp"

namespace roboticslab {

class LineCallbackPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
    protected:
        void onRead(yarp::os::Bottle& b);
        SharedArea* sharedArea;

    public:
        void setSharedArea(SharedArea* _sharedArea);
};

}  // namespace roboticslab

#endif  // __LINE_CALLBACK_PORT_HPP__

