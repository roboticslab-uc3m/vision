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
    printf("Got: %s %s\n",b1->toString().c_str(),b2->toString().c_str());

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
