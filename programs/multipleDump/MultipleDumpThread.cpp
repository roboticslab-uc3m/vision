// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MultipleDumpThread.hpp"

#include <yarp/os/Time.h>

namespace roboticslab
{

/************************************************************************/
void MultipleDumpThread::run() {
    // printf("[MultipleDumpThread] run()\n");

    yarp::os::Bottle* b1 = in1->read(false);
    if (b1 == NULL) {
        //printf("No b1 yet...\n");
        return;
    }
    yarp::os::Bottle* b2 = in2->read(false);
    if (b2 == NULL) {
        //printf("No b2 yet...\n");
        return;
    }
    if(!firstTimeTaken){
        firstTimeTaken = true;
        firstTime = yarp::os::Time::now();
    }

    double x1 = b1->get(0).asDouble();
    double y1 = b1->get(1).asDouble();
    double z1 = b1->get(2).asDouble();
    double x2 = b2->get(0).asDouble();
    double y2 = b2->get(1).asDouble();
    double z2 = b2->get(2).asDouble();

    //printf("%f %s %s\n",Time::now(),b1->toString().c_str(),b2->toString().c_str());
    fprintf(filePtr,"%f %s %s %f %f %f %f %f %f %f\n",yarp::os::Time::now()-firstTime,
            b1->toString().c_str(),b2->toString().c_str(),
            x1-x2, y1-y2, z1-z2,
            fabs(x1-x2), fabs(y1-y2), fabs(z1-z2),
            sqrt( pow((x1-x2),2) + pow((y1-y2),2) + pow((z1-z2),2) )
            );

}

/************************************************************************/

void MultipleDumpThread::setFilePtr(FILE *value) {
    filePtr = value;
}

/************************************************************************/

void MultipleDumpThread::setIn1(yarp::os::BufferedPort<yarp::os::Bottle> *value) {
    in1 = value;
}

/************************************************************************/

void MultipleDumpThread::setIn2(yarp::os::BufferedPort<yarp::os::Bottle> *value) {
    in2 = value;
}

/************************************************************************/

}  // namespace roboticslab
