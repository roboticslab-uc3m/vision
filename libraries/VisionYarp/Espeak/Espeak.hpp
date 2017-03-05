// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ESPEAK_HPP__
#define __ESPEAK_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>

#include <iostream> // only windows

#include <speak_lib.h>

#include <Speech_IDL.h>

#include "ColorDebug.hpp"

#define DEFAULT_NAME "/espeak"
#define DEFAULT_VOICE "mb-en1"

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

class Espeak : public yarp::dev::DeviceDriver, public Speech_IDL {

    public:

        Espeak();

        virtual bool setLanguage(const std::string& language);

        virtual std::vector<std::string> getSupportedLang();

        virtual bool say(const std::string& text);

        virtual bool setSpeed(const int16_t speed);

        virtual bool setPitch(const int16_t pitch);

        virtual int16_t getSpeed();

        virtual int16_t getPitch();

        virtual bool play();

        virtual bool stop();

        virtual bool pause()
        {
            return false;
        }

        // -------- DeviceDriver declarations.  --------

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

    private:

        void printError(espeak_ERROR code);

        /*
           FROM speak_lib.h :

           output: the audio data can either be played by eSpeak or passed back by the SynthCallback function.

           Buflength:  The length in mS of sound buffers passed to the SynthCallback function.

           options: bit 0: 1=allow espeakEVENT_PHONEME events.

           path: The directory which contains the espeak-data directory, or NULL for the default location.

           espeak_Initialize() Returns: sample rate in Hz, or -1 (EE_INTERNAL_ERROR).

           text: The text to be spoken, terminated by a zero character. It may be either 8-bit characters,
              wide characters (wchar_t), or UTF8 encoding.  Which of these is determined by the "flags"
              parameter.

           Size: Equal to (or greatrer than) the size of the text data, in bytes.  This is used in order
              to allocate internal storage space for the text.  This value is not used for
              AUDIO_OUTPUT_SYNCHRONOUS mode.

           position:  The position in the text where speaking starts. Zero indicates speak from the
              start of the text.

           position_type:  Determines whether "position" is a number of characters, words, or sentences.
              Values:

           end_position:  If set, this gives a character position at which speaking will stop.  A value
              of zero indicates no end position.

           flags:  These may be OR'd together:
              Type of character codes, one of:
                 espeakCHARS_UTF8     UTF8 encoding
                 espeakCHARS_8BIT     The 8 bit ISO-8859 character set for the particular language.
                 espeakCHARS_AUTO     8 bit or UTF8  (this is the default)
                 espeakCHARS_WCHAR    Wide characters (wchar_t)

              espeakSSML   Elements within < > are treated as SSML elements, or if not recognised are ignored.

              espeakPHONEMES  Text within [[ ]] is treated as phonemes codes (in espeak's Hirshenbaum encoding).

              espeakENDPAUSE  If set then a sentence pause is added at the end of the text.  If not set then
                 this pause is suppressed.

           unique_identifier: message identifier; helpful for identifying later
             data supplied to the callback.

           user_data: pointer which will be passed to the callback function.

           espeak_Synth() Returns: EE_OK: operation achieved
                                   EE_BUFFER_FULL: the command can not be buffered;
                                   you may try after a while to call the function again.
                                   EE_INTERNAL_ERROR.
        */

        espeak_POSITION_TYPE position_type;
        espeak_AUDIO_OUTPUT output;
        char *path;
        int buflength, options;
        void* user_data;
        t_espeak_callback *synthCallback;
        espeak_PARAMETER Param;
        unsigned int size, position, end_position, flags, *unique_identifier;

        // YARP
        yarp::os::RpcServer rpcPort;
};

}  // namespace teo

#endif  // __ESPEAK_HPP__

