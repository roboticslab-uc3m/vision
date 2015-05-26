// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * ColorDebug
 * Version: 0.4 - __func__ as __FUNCTION__ for VS2008.
 * Version: 0.3 - Added CD_*****_NO_HEADER. Added scopes for each CD_printf macro.
 * Version: 0.2 - Added CD_PERROR.
 * Version: 0.1 - Initial version.
 *
 * Copyright: UC3M 2014 (C)
 * Author:
 * <a href="http://roboticslab.uc3m.es/roboticslab/people/jg-victores">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, look for LGPL.TXT
 */

#ifndef __COLOR_DEBUG_HPP__
#define __COLOR_DEBUG_HPP__

#include <stdio.h>
#include <string>  // std::string
#include <string.h>  // strrchr

namespace ColorDebug {

//-- Thanks tomlogic @ http://stackoverflow.com/questions/2281970/cross-platform-defining-define-for-macros-function-and-func
#if defined ( WIN32 )
#define __func__ __FUNCTION__
#endif

//-- Thanks dalle @ http://stackoverflow.com/questions/1546789/clean-code-to-printf-size-t-in-c-or-nearest-equivalent-of-c99s-z-in-c
#if defined(_MSC_VER)
  #define CD_SIZE_T    "%Iu"
  #define CD_SSIZE_T   "%Id"
  #define CD_PTRDIFF_T "%Id"
#elif defined(__GNUC__)
  #define CD_SIZE_T    "%zu"
  #define CD_SSIZE_T   "%zd"
  #define CD_PTRDIFF_T "%zd"
#else
  // TODO figure out which to use.
  #if NUMBITS == 32
    #define CD_SIZE_T_SPECIFIER    something_unsigned
    #define CD_SSIZE_T_SPECIFIER   something_signed
    #define CD_PTRDIFF_T_SPECIFIER something_signed
  #else
    #define CD_SIZE_T_SPECIFIER    something_bigger_unsigned
    #define CD_SSIZE_T_SPECIFIER   something_bigger_signed
    #define CD_PTRDIFF_T_SPECIFIER something-bigger_signed
  #endif
#endif

//-- Thanks red1ynx @ http://stackoverflow.com/questions/8487986/file-macro-shows-full-path
#define __REL_FILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
// For Windows use '\\' instead of '/'.

// http://stackoverflow.com/questions/1961209/making-some-text-in-printf-appear-in-green-and-red
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

// http://en.wikipedia.org/wiki/Variadic_macro
// http://stackoverflow.com/questions/15549893/modify-printfs-via-macro-to-include-file-and-line-number-information
#define CD_PERROR(...) {fprintf(stderr,RED); do{fprintf(stderr, "[error] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
                         fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr, "[error] "); perror(""); fprintf(stderr,RESET);}

#define CD_ERROR(...) {fprintf(stderr,RED); do{fprintf(stderr, "[error] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
                         fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr,RESET);}

#define CD_WARNING(...) {fprintf(stderr,YELLOW); do{fprintf(stderr, "[warning] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
                           fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr,RESET);}

#define CD_SUCCESS(...) {fprintf(stderr,GREEN); do{printf("[success] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
                           printf(__VA_ARGS__);} while(0); fprintf(stderr,RESET);}

#define CD_INFO(...) {do{printf("[info] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
                           printf(__VA_ARGS__);} while(0);}

#define CD_DEBUG(...) {fprintf(stderr,BLUE); do{printf("[debug] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
                           printf(__VA_ARGS__);} while(0); fprintf(stderr,RESET);}


#define CD_ERROR_NO_HEADER(...) {fprintf(stderr,RED); fprintf(stderr, __VA_ARGS__); fprintf(stderr,RESET);}

#define CD_WARNING_NO_HEADER(...) {fprintf(stderr,YELLOW); fprintf(stderr, __VA_ARGS__); fprintf(stderr,RESET);}

#define CD_SUCCESS_NO_HEADER(...) {fprintf(stderr,GREEN); printf(__VA_ARGS__); fprintf(stderr,RESET);}

#define CD_INFO_NO_HEADER(...) {do{printf("[info] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
                           printf(__VA_ARGS__);} while(0);}

#define CD_DEBUG_NO_HEADER(...) {fprintf(stderr,BLUE); printf(__VA_ARGS__); fprintf(stderr,RESET);}





} //ColorDebug

#endif  // __COLOR_DEBUG_HPP__

