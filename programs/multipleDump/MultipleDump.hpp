// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MULTIPLE_DUMP_HPP__
#define __MULTIPLE_DUMP_HPP__

#include <sstream>

#include "MultipleDumpThread.hpp"

#define DEFAULT_WATCHDOG    2       // [s]


namespace roboticslab
{

/**
 * @ingroup multipleDump
 *
 * @brief Dump from multiple sources.
 */
class MultipleDump : public yarp::os::RFModule {

    public:
        bool configure(yarp::os::ResourceFinder &rf);

    protected:
        MultipleDumpThread multipleDumpThread;

        bool interruptModule();
        double getPeriod();
        bool updateModule();
        double watchdog;

        yarp::os::BufferedPort<yarp::os::Bottle> in1;
        yarp::os::BufferedPort<yarp::os::Bottle> in2;

        FILE * filePtr;
};

}  // namespace roboticslab

#endif  // __MULTIPLE_DUMP_HPP__

