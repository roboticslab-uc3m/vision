// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Espeak.hpp"

// -----------------------------------------------------------------------------

teo::Espeak::Espeak()
{   
    path = NULL;
    buflength = 500;
    options = 0;
    position = 0;
    end_position = 0;
    flags = espeakCHARS_AUTO | espeakENDPAUSE;
    output = AUDIO_OUTPUT_PLAYBACK;
    espeak_Initialize(output, buflength, path, options);
    voice = "default"; //-- mbrola voices (mb-en1) / "default"
}

// -----------------------------------------------------------------------------
bool teo::Espeak::say(const std::string& text)
{
    espeak_SetVoiceByName(voice);   
    size = strlen(text.c_str())+1;
    printf("Going to say: %s\n", text.c_str());
    espeak_Synth( text.c_str(), size, position, position_type, end_position, flags, unique_identifier, user_data );
    espeak_Synchronize();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::Espeak::setSpeed(const int16_t speed)
{
    bool result = false;
    espeak_ERROR error = espeak_SetParameter(espeakRATE, speed, 0); // -- EE_OK=0, EE_INTERNAL_ERROR=-1, EE_BUFFER_FULL=1, EE_NOT_FOUND=2
    if (error == 0 )
        result = true;
    if (error == -1)
        CD_ERROR("EE_INTERNAL_ERROR\n");
    if (error == 1)
        CD_ERROR("EE_BUFFER_FULL\n");
    if (error == 2)
        CD_ERROR("EE_NOT_FOUND\n");

    return result;
}

// -----------------------------------------------------------------------------

 bool teo::Espeak::setPitch(const int16_t pitch)
 {
    bool result = false;
    espeak_ERROR error = espeak_SetParameter(espeakPITCH, pitch, 0);  // -- EE_OK=0, EE_INTERNAL_ERROR=-1, EE_BUFFER_FULL=1, EE_NOT_FOUND=2
    if (error == 0 )
        result = true;
    if (error == -1)
        CD_ERROR("EE_INTERNAL_ERROR\n");
    if (error == 1)
        CD_ERROR("EE_BUFFER_FULL\n");
    if (error == 2)
        CD_ERROR("EE_NOT_FOUND\n");

    return result;
 }

 // -----------------------------------------------------------------------------

 int16_t teo::Espeak::getSpeed()
 {
    return espeak_GetParameter(espeakRATE, 1);
 }

 // -----------------------------------------------------------------------------

 int16_t teo::Espeak::getPitch()
 {
    return espeak_GetParameter(espeakPITCH, 1);
 }

 // -----------------------------------------------------------------------------

 bool teo::Espeak::play()
 {
    bool result = false;
    if( espeak_Synchronize() == 0) // -- EE_OK=0: operation achieved, EE_INTERNAL_ERROR=-1
        result = true;
    else
        CD_ERROR("EE_INTERNAL_ERROR\n");
    return result;
 }

 // -----------------------------------------------------------------------------

 bool teo::Espeak::stop()
 {
    bool result = false;
    if(espeak_Cancel() == 0)  // -- EE_OK=0: operation achieved, EE_INTERNAL_ERROR=-1
        result = true;
    else
        CD_ERROR("EE_INTERNAL_ERROR\n");
    return result;
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
