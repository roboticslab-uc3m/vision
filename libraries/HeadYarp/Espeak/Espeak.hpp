// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ESPEAK_HPP__
#define __ESPEAK_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>

#include <iostream> // only windows

#include "ColorDebug.hpp"

//#define DEFAULT_NUM_LINKS 1  // int

namespace teo
{

/**
 * @ingroup TeoYarp
 * \defgroup Espeak
 *
 * @brief Contains teo::Espeak.
 */

/**
 * @ingroup Espeak
 * @brief The Espeak class implements...
 */

class Espeak : public yarp::dev::DeviceDriver {

    public:

        Espeak() {}

        // -- ??? declarations. Implementation in ???Impl.cpp--

        // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

        /**
        * Open the DeviceDriver.
        * @param config is a list of parameters for the device.
        * Which parameters are effective for your device can vary.
        * See \ref dev_examples "device invocation examples".
        * If there is no example for your device,
        * you can run the "yarpdev" program with the verbose flag
        * set to probe what parameters the device is checking.
        * If that fails too,
        * you'll need to read the source code (please nag one of the
        * yarp developers to add documentation for your device).
        * @return true/false upon success/failure
        */
        virtual bool open(yarp::os::Searchable& config);

        /**
        * Close the DeviceDriver.
        * @return true/false on success/failure.
        */
        virtual bool close();

    protected:

};

}  // namespace teo

#endif  // __ESPEAK_HPP__

