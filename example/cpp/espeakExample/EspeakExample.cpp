// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EspeakExample.hpp"

namespace teo
{

bool EspeakExample::run()
{
    if (!yarp::os::Network::checkNetwork())
    {
        printf("Please start a yarp name server first\n");
        return(1);
    }

    //Configure Drivers
    yarp::os::Property options; //create an instance of Property, a nice YARP class for storing name-value (key-value) pairs
    options.put("device","Espeak"); //we add a name-value pair that indicates the YARP device
    dd.open(options); //Configure the YARP multi-use driver with the given options

    if(!dd.isValid())
    {
      printf("Device not available.\n");
	  dd.close();
      yarp::os::Network::fini(); //disconnect from the YARP network
      return 1;
    }

    /*bool ok = dd.view(pos); // connect 'pos' interface to 'dd' device
    if (!ok)
    {
        printf("[warning] Problems acquiring robot interface\n");
        return false;
    }
    else
        printf("[success] testAsibot acquired robot interface\n");
    */

    //yarp::os::Time::delay(5);

    dd.close();

    return 0;
}

} //namespace TEO
