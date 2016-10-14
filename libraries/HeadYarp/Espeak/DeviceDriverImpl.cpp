// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Espeak.hpp"

// -----------------------------------------------------------------------------

teo::Espeak::Espeak()
{
    //--RAUL
}

// -----------------------------------------------------------------------------
// ---- TEST-----
//don't delete this callback function.
int SynthCallback(short *wav, int numsamples, espeak_EVENT *events)
{

    return 0;
}

bool teo::Espeak::say(const std::string& text)
{
    //--RAUL
    printf("Going to say: %s\n", text.c_str());
    // -- TEST
    espeak_ERROR speakErr;

        //must be called before any other functions
        //espeak initialize
        if(espeak_Initialize(AUDIO_OUTPUT_SYNCH_PLAYBACK,0,NULL,espeakINITIALIZE_PHONEME_EVENTS) <0)
        {
            puts("could not initialize espeak\n");
            return -1;
        }

        espeak_SetSynthCallback(SynthCallback);

        //make some text to spit out
        char textBuff[255]={0};

        strcpy(textBuff, "hello, hello, hello world");
        /* ESPEAK_API espeak_ERROR espeak_Synth(
        const void *text,
        size_t size,
        unsigned int position,
        espeak_POSITION_TYPE position_type,
        unsigned int end_position,
        unsigned int flags,
        unsigned int* unique_identifier,
        void* user_data);
        */
        if((speakErr=espeak_Synth(textBuff, strlen(textBuff), 0,POS_CHARACTER,0,espeakCHARS_AUTO,NULL,NULL))!= EE_OK)
        {
            puts("error on synth creation\n");

        }

    return true;
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
