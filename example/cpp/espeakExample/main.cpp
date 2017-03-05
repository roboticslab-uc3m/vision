// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup teo_examples_cpp
 * \defgroup espeakExample espeakExample
 *
 * @brief This example...
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2016 Universidad Carlos III de Madrid;
 *
 * Authors: rsantos88, jgvictores
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd $ROBOTICSLAB_VISION_ROOT/example/cpp
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * <b>Running</b>
\verbatim
./espeakExample
\endverbatim
 *
 */

#include "EspeakExample.hpp"

//YARP_DECLARE_PLUGINS(HeadYarp)

int main(int argc, char **argv)
{
    //YARP_REGISTER_PLUGINS(HeadYarp);

    teo::EspeakExample mod;
    return mod.run();
}

