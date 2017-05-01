// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SHARED_AREA_HPP__
#define __SHARED_AREA_HPP__

#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

namespace teo {

class SharedArea {
    private:
        double lineCoords[6];
        yarp::os::Semaphore lcMutex;

    public:
        void init();
        void setLC(const double _lineCoords[6]);
        void getLC(double _lineCoords[6]);
        void getLongLC(double _longLineCoords[6]);
};

}  // teo

#endif  // __SHARED_AREA_HPP__

