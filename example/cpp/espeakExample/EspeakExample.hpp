// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ESPEAK_EXAMPLE_HPP__
#define __ESPEAK_EXAMPLE_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

namespace teo
{

class EspeakExample
{
public:
    bool run();

private:
    yarp::os::Network yarp; // connect to YARP network
    yarp::dev::PolyDriver dd; //create a YARP multi-use driver
};

}  // namespace teo

#endif // __ESPEAK_EXAMPLE_HPP__

