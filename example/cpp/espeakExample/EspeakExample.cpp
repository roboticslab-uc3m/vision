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
        printf("[warning] Problems acquiring espeak interface\n");
        return false;
    }
    printf("[success] EspeakExample acquired espeak interface\n");

    espeak->setSpeed(150);  // Values 80 to 450.
    espeak->setPitch(60);   // 50 = normal
    printf("Running with (%i) of speed\n", espeak->getSpeed());
    printf("Running with (%i) of spitch\n", espeak->getPitch());
    espeak->say("Hello, my name is Teo. I want to follow you. Please, tell me. Ok, I will follow you. Ok, I will stop following you.");    

    dd.close();

    return 0;
}

} //namespace TEO
