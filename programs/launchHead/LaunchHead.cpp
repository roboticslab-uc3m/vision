// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchHead.hpp"

/************************************************************************/
teo::LaunchHead::LaunchHead() { }

/************************************************************************/
bool teo::LaunchHead::configure(ResourceFinder &rf) {

    printf(BOLDBLUE);

    kinect = rf.check("kinect",Value(DEFAULT_KINECT),"Kinect on or off").asString();
    if( kinect == "on" )
        printf("\t--kinect on (Kinect on or off)\n" );
    else
        printf("\t--kinect off (Kinect on or off)\n" );

    printf(RESET);

    if( kinect == "on" )
    {
        Property kinectOptions;
        kinectOptions.put("device","KinectDeviceLocal");
        kinectOptions.put("portPrefix","/teo");
        kinectDevice.open(kinectOptions);

        if (!kinectDevice.isValid())
        {
            CD_ERROR("Kinect class instantiation not worked.\n");
            // kinectDevice.close();  // un-needed?
            return false;
        }
    }

    Property motorOptions;
    motorOptions.fromString(rf.toString());
    motorOptions.put("name","/teo/head");
    motorOptions.put("device","controlboard");
    motorOptions.put("subdevice","headbot");

    motorDevice.open(motorOptions);
    
    if (!motorDevice.isValid()) {
        CD_ERROR("headDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_HeadYarp_headbot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_headbot is set\" --> should be --> \"ENABLE_headbot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    return true;
}

/************************************************************************/

bool teo::LaunchHead::updateModule() {
    //printf("LaunchHead alive...\n");
    return true;
}

/************************************************************************/

bool teo::LaunchHead::close() {

    motorDevice.close();

    if( kinect == "on" )
    {
        kinectDevice.close();
    }

    return true;
}

/************************************************************************/
