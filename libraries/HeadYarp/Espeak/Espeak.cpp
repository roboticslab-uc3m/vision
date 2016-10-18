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
}

// -----------------------------------------------------------------------------

void teo::Espeak::printError(espeak_ERROR code)
{
    if (code == EE_INTERNAL_ERROR)
        CD_ERROR("EE_INTERNAL_ERROR\n");
    if (code == EE_BUFFER_FULL)
        CD_ERROR("EE_BUFFER_FULL\n");
    if (code == EE_NOT_FOUND)
        CD_ERROR("EE_NOT_FOUND\n");
}

// -----------------------------------------------------------------------------

bool teo::Espeak::setLanguage(const std::string& language)
{
    espeak_ERROR ret = espeak_SetVoiceByName(language.c_str());
    if ( ret != EE_OK)
    {
        CD_ERROR("%s\n",language.c_str());
        printError(ret);
        return false;
    }
    CD_SUCCESS("%s\n",language.c_str());
    return true;
}

// -----------------------------------------------------------------------------

std::vector<std::string> teo::Espeak::getSupportedLang()
{
    const espeak_VOICE** list = espeak_ListVoices(NULL);
    std::vector<std::string> supportedLang;
    int i = 0;
    while (list[i] != NULL)
    {
        CD_DEBUG("%s\n",list[i]->name);
        supportedLang.push_back(list[i]->name);
        i++;
    }

    //-- Hard-coded for now
    /*std::string mb_en1("mb-en1");
    supportedLang.push_back(mb_en1);
    std::string mb_es1("mb-es1");
    supportedLang.push_back(mb_es1);*/

    return supportedLang;
}

// -----------------------------------------------------------------------------
bool teo::Espeak::say(const std::string& text)
{
    espeak_ERROR ret = espeak_Synth( static_cast<const void*>(text.c_str()), text.length(), position, position_type, end_position, flags, unique_identifier, user_data );
    if ( ret != EE_OK)
    {
        CD_ERROR("%s\n", text.c_str());
        printError(ret);
        return false;
    }
    //espeak_Synchronize();  //-- Just for blocking

    CD_SUCCESS("%s\n", text.c_str());
    return true;
}

// -----------------------------------------------------------------------------

bool teo::Espeak::setSpeed(const int16_t speed)
{
    espeak_ERROR ret = espeak_SetParameter(espeakRATE, speed, 0);
    if ( ret != EE_OK)
    {
        CD_ERROR("%d\n",speed);
        printError(ret);
        return false;
    }
    CD_SUCCESS("%d\n",speed);
    return true;
}

// -----------------------------------------------------------------------------

 bool teo::Espeak::setPitch(const int16_t pitch)
 {
    espeak_ERROR ret = espeak_SetParameter(espeakPITCH, pitch, 0);
    if ( ret != EE_OK)
    {
        CD_ERROR("%d\n",pitch);
        printError(ret);
        return false;
    }
    CD_SUCCESS("%d\n",pitch);
    return true;
 }

 // -----------------------------------------------------------------------------

 int16_t teo::Espeak::getSpeed()
 {
     CD_DEBUG("\n");
     return espeak_GetParameter(espeakRATE, 1);
 }

 // -----------------------------------------------------------------------------

int16_t teo::Espeak::getPitch()
{
    CD_DEBUG("\n");
    return espeak_GetParameter(espeakPITCH, 1);
}

 // -----------------------------------------------------------------------------

bool teo::Espeak::play()
{
    //-- Wolud need to store previous say().
    CD_WARNING("Not implemented. Use say().\n");
    return false;
}

 // -----------------------------------------------------------------------------

 bool teo::Espeak::stop()
 {
     espeak_ERROR ret = espeak_Cancel();
     if ( ret != EE_OK)
     {
         CD_ERROR("\n");
         printError(ret);
         return false;
     }
     CD_SUCCESS("\n");
     return true;
 }


// ------------------- DeviceDriver Related ------------------------------------

bool teo::Espeak::open(yarp::os::Searchable& config)
{
    std::string name = config.check("name",yarp::os::Value(DEFAULT_NAME),"port /name (auto append of /rpc:s)").asString();
    std::string voice = config.check("voice",yarp::os::Value(DEFAULT_VOICE),"voice").asString();

    CD_DEBUG_NO_HEADER("--name: %s [%s]\n",name.c_str(),DEFAULT_NAME);
    CD_DEBUG_NO_HEADER("--voice: %s [%s]\n",voice.c_str(),DEFAULT_VOICE);

    int ret = espeak_Initialize(output, buflength, path, options);
    if(ret == EE_INTERNAL_ERROR)
        return false;

    if( ! setLanguage(voice) )
        return false;

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
