// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MultipleDumpThread.hpp"

/************************************************************************/
void MultipleDumpThread::run() {
    // printf("[MultipleDumpThread] run()\n");

    Bottle* b1 = in1->read(false);
    if (b1 == NULL) {
        //printf("No b1 yet...\n");
        return;
    }
    Bottle* b2 = in2->read(false);
    if (b2 == NULL) {
        //printf("No b2 yet...\n");
        return;
    }
    if(!firstTimeTaken){
        firstTimeTaken = true;
        firstTime = Time::now();
    }

    //printf("%f %s %s\n",Time::now(),b1->toString().c_str(),b2->toString().c_str());
    fprintf(filePtr,"%f %s %s %f %f %f\n",Time::now()-firstTime,
            b1->toString().c_str(),b2->toString().c_str(),
            b1->get(0).asDouble()-b2->get(0).asDouble(),
            b1->get(1).asDouble()-b2->get(1).asDouble(),
            b1->get(2).asDouble()-b2->get(2).asDouble()
            );

}

/************************************************************************/

void MultipleDumpThread::setFilePtr(FILE *value) {
    filePtr = value;
}

/************************************************************************/

void MultipleDumpThread::setIn1(BufferedPort<Bottle> *value) {
    in1 = value;
}

/************************************************************************/

void MultipleDumpThread::setIn2(BufferedPort<Bottle> *value) {
    in2 = value;
}

/************************************************************************/
