// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SHARED_AREA_HPP__
#define __SHARED_AREA_HPP__

#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

namespace pointatobject {

class SharedArea {
    private:
        double lineCoords[6];
        Semaphore lcMutex;

    public:
        void init();
        void setLC(const double _lineCoords[6]);
        void getLC(double _lineCoords[6]);
        void getLongLC(double _longLineCoords[6]);
};

}  // pointatobject

#endif  // __SHARED_AREA_HPP__

