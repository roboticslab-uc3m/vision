// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MULTIPLE_DUMP_THREAD_HPP__
#define __MULTIPLE_DUMP_THREAD_HPP__

#include <math.h>  // fabs

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>

#define DEFAULT_RATE_MS 20

using namespace yarp::os;

namespace teo
{

class MultipleDumpThread : public RateThread {

    public:

        MultipleDumpThread() : RateThread(DEFAULT_RATE_MS), firstTimeTaken(false) {}

        virtual void run();

        void setFilePtr(FILE *value);
        void setIn1(BufferedPort<Bottle> *value);
        void setIn2(BufferedPort<Bottle> *value);

protected:

        FILE * filePtr;
        BufferedPort<Bottle>* in1;
        BufferedPort<Bottle>* in2;
        double firstTime;
        bool firstTimeTaken;

};

}  // namespace teo

#endif  // __MULTIPLE_DUMP_THREAD_HPP__

