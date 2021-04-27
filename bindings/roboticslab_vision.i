// Copyright:
// Author:
// CopyPolicy:

//////////////////////////////////////////////////////////////////////////
//
// This is a configuration file to explain VISION to SWIG
//
// SWIG, for the most part, understands VISION auto-magically.
// There are a few things that need to be explained:
//  + use of multiple inheritance
//  + use of names that clash with special names in Java/Python/Perl/...
//  + use of templates

%module roboticslab_vision

%{
#include "yarp/os/Property.h"
%}

%{
/* Includes the header in the wrapper code */
#include "IDetector.hpp"
%}

/* Parse the header file to generate wrappers */
%include "IDetector.hpp"

%{
#include <yarp/dev/all.h>
roboticslab::IDetector *viewIDetector(yarp::dev::PolyDriver& d)
{
    roboticslab::IDetector *result;
    d.view(result);
    return result;
}
%}
extern roboticslab::IDetector *viewIDetector(yarp::dev::PolyDriver& d);
