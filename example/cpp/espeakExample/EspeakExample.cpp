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

    teo::Espeak* espeak;
    bool ok = dd.view(espeak); // connect interface to 'dd' device
    if (!ok)
    {
        printf("[warning] Problems acquiring interface\n");
        return false;
    }
    printf("[success] testAsibot acquired robot interface\n");

    espeak->say("Hello, my name is Teo. I want to follow you. Please, tell me. Ok, I will follow you. Ok, I will stop following you.");
    //yarp::os::Time::delay(5);

    dd.close();

    return 0;
}

} //namespace TEO
