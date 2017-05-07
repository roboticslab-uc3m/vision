#ifndef __SHARED_AREA_HPP__
#define __SHARED_AREA_HPP__

#include <stdio.h>

#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

namespace roboticslab {

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

}  // namespace roboticslab

#endif  // __SHARED_AREA_HPP__

