// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Espeak.hpp"

// -----------------------------------------------------------------------------

teo::Espeak::Espeak()
{   
    path = NULL;
    Buflength = 500;
    Options = 0;
    position = 0;
    end_position = 0;
    flags = espeakCHARS_AUTO | espeakENDPAUSE;
    voice = "mb-en1"; //-- mbrola voices / "default"    
}

// -----------------------------------------------------------------------------
bool teo::Espeak::say(const std::string& text)
{
    output = AUDIO_OUTPUT_PLAYBACK;
    int I, Run = 1, L;
    espeak_Initialize(output, Buflength, path, Options );    
    espeak_SetVoiceByName(voice);   
    Size = strlen(text.c_str())+1;
    printf("Going to say: %s\n", text.c_str());
    espeak_Synth( text.c_str(), Size, position, position_type, end_position, flags, unique_identifier, user_data );
    espeak_Synchronize( );
    printf("\n:Done\n");

    return true;
}

// -----------------------------------------------------------------------------

bool setSpeed(const int16_t speed)
{
    espeak_SetParameter(espeakRATE, speed, 0);
}

// -----------------------------------------------------------------------------

 bool setPitch(const int16_t pitch)
 {
    espeak_SetParameter(espeakPITCH, pitch, 0);
 }

 // -----------------------------------------------------------------------------

 int16_t getSpeed()
 {
    return espeak_GetParameter(espeakRATE, 1) ;
 }

 // -----------------------------------------------------------------------------

 int16_t getPitch()
 {
    return espeak_GetParameter(espeakPITCH, 1) ;
 }


// ------------------- DeviceDriver Related ------------------------------------

bool teo::Espeak::open(yarp::os::Searchable& config)
{
    //numLinks = config.check("numLinks",yarp::os::Value(DEFAULT_NUM_LINKS),"chain number of segments").asInt();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::Espeak::close()
{
    return true;
}

// -----------------------------------------------------------------------------
