// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Espeak.hpp"

// -----------------------------------------------------------------------------

teo::Espeak::Espeak()
{   
    path = NULL;
    buflength = 500;
    options = 0;
    position = 0;
    position_type = POS_CHARACTER;
    end_position = 0;
    flags = espeakCHARS_AUTO | espeakENDPAUSE;
    unique_identifier = NULL;
    user_data = NULL;
    output = AUDIO_OUTPUT_PLAYBACK;
    espeak_Initialize(output, buflength, path, options);
}

// -----------------------------------------------------------------------------

bool teo::Espeak::setLanguage(const std::string& language)
{
    espeak_SetVoiceByName(language.c_str());

    return true;
}

// -----------------------------------------------------------------------------

std::vector<std::string> teo::Espeak::getSupportedLang()
{
    std::vector<std::string> supportedLang;
    //-- Hard-coded for now
    std::string mb_en1("mb-en1");
    supportedLang.push_back(mb_en1);
    std::string mb_es1("mb-es1");
    supportedLang.push_back(mb_es1);
    return supportedLang;
}

// -----------------------------------------------------------------------------
bool teo::Espeak::say(const std::string& text)
{
    CD_DEBUG("Going to say: %s\n", text.c_str());

    espeak_ERROR error = espeak_Synth( static_cast<const void*>(text.c_str()), text.length(), position, position_type, end_position, flags, unique_identifier, user_data );
    //espeak_Synchronize();  //-- Just for blocking

    if (error == 0 )
        return true;
    if (error == EE_INTERNAL_ERROR)
        CD_ERROR("EE_INTERNAL_ERROR\n");
    if (error == EE_BUFFER_FULL)
        CD_ERROR("EE_BUFFER_FULL\n");
    if (error == EE_NOT_FOUND)
        CD_ERROR("EE_NOT_FOUND\n");

    return false;
}

// -----------------------------------------------------------------------------

bool teo::Espeak::setSpeed(const int16_t speed)
{
    espeak_ERROR error = espeak_SetParameter(espeakRATE, speed, 0); // -- EE_OK=0, EE_INTERNAL_ERROR=-1, EE_BUFFER_FULL=1, EE_NOT_FOUND=2
    if (error == 0 )
    {
        CD_SUCCESS("%d\n",speed);
        return true;
    }
    if (error == EE_INTERNAL_ERROR)
        CD_ERROR("EE_INTERNAL_ERROR\n");
    if (error == EE_BUFFER_FULL)
        CD_ERROR("EE_BUFFER_FULL\n");
    if (error == EE_NOT_FOUND)
        CD_ERROR("EE_NOT_FOUND\n");

    return false;
}

// -----------------------------------------------------------------------------

 bool teo::Espeak::setPitch(const int16_t pitch)
 {

    espeak_ERROR error = espeak_SetParameter(espeakPITCH, pitch, 0);  // -- EE_OK=0, EE_INTERNAL_ERROR=-1, EE_BUFFER_FULL=1, EE_NOT_FOUND=2
    if (error == 0 )
    {
        CD_SUCCESS("%d\n",pitch);
        return true;
    }
    if (error == EE_INTERNAL_ERROR)
        CD_ERROR("EE_INTERNAL_ERROR\n");
    if (error == EE_BUFFER_FULL)
        CD_ERROR("EE_BUFFER_FULL\n");
    if (error == EE_NOT_FOUND)
        CD_ERROR("EE_NOT_FOUND\n");

    return false;
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
     CD_DEBUG("\n");
     bool result = espeak_Cancel();  // -- EE_OK=0: operation achieved, EE_INTERNAL_ERROR=-1
     if ( result == EE_OK)
        return true;

     CD_ERROR("EE_INTERNAL_ERROR\n");
     return false;
 }


// ------------------- DeviceDriver Related ------------------------------------

bool teo::Espeak::open(yarp::os::Searchable& config)
{
    std::string name = config.check("name",yarp::os::Value(DEFAULT_NAME),"port /name (auto append of /rpc:s)").asString();
    std::string voice = config.check("voice",yarp::os::Value(DEFAULT_VOICE),"voice").asString();

    CD_DEBUG_NO_HEADER("--name: %s [%s]\n",name.c_str(),DEFAULT_NAME);
    CD_DEBUG_NO_HEADER("--voice: %s [%s]\n",voice.c_str(),DEFAULT_VOICE);

    setLanguage(voice);

    name += "/rpc:s";

    this->yarp().attachAsServer(rpcPort);

    if( ! rpcPort.open(name.c_str()) )
    {
          CD_ERROR("Cannot open port %s\n",name.c_str());
          return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::Espeak::close()
{
    return true;
}

// -----------------------------------------------------------------------------
