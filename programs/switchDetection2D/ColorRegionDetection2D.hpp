// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTION_2D_HPP__
#define __COLOR_REGION_DETECTION_2D_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Image.h>

//#include "highgui.h" // to show windows

#include <ColorDebug.h>

#include "TravisLib.hpp"

#include "Transformation.hpp"

#define DEFAULT_ALGORITHM "blueMinusRed"
#define DEFAULT_LOCATE "centroid"
#define DEFAULT_MAX_NUM_BLOBS 1
#define DEFAULT_MORPH_CLOSING 2
#define DEFAULT_OUT_FEATURES "locX locY angle area"  // it's a bottle!!
#define DEFAULT_OUT_FEATURES_FORMAT 0
#define DEFAULT_OUT_IMAGE 1
#define DEFAULT_RATE_MS 20
#define DEFAULT_SEE_BOUNDING 3
#define DEFAULT_THRESHOLD 55

namespace roboticslab
{

class ColorRegionDetectionTransformation : public Transformation
{
public:
    ColorRegionDetectionTransformation(yarp::os::Searchable* parameters);
    ColorRegionDetectionTransformation() : area(-1), hue_peak(-1), hue_mode(-1), hue_mean(-1), hue_stddev(-1), saturation_peak(-1),
        saturation_mean(-1), saturation_stddev(-1), value_peak(-1), value_mode(-1), value_mean(-1), value_stddev(-1), locX(-1), locY(-1),
        rectangularity(-1), axisFirst(-1), axisSecond(-1), aspectRatio(-1), solidity(-1), massCenterlocX(-1), massCenterlocY(-1),
        arc(-1), radius(-1) {
        std::printf("\t--algorithm (redMinusBlue,greenMinusRed...; default: \"%s\")\n",algorithm.c_str());
        std::printf("\t--locate (centroid,bottom; default: \"%s\")\n",locate.c_str());
        std::printf("\t--morphClosing (percentage, 2 or 4 okay; default: \"%f\")\n",morphClosing);
        std::printf("\t--maxNumBlobs (default: \"%d\")\n",maxNumBlobs);
        std::printf("\t--outFeatures (default: \"(%s)\")\n",outFeatures.toString().c_str());
        std::printf("\t--outFeaturesFormat (0=bottled,1=minimal; default: \"%d\")\n",outFeaturesFormat);
        std::printf("\t--outImage (0=rgb,1=bw; default: \"%d\")\n",outImage);
        std::printf("\t--seeBounding (0=none,1=contour,2=box,3=both; default: \"%d\")\n",seeBounding);
        std::printf("\t--threshold (default: \"%d\")\n",threshold);

        /*if (rf.check("algorithm")) algorithm = rf.find("algorithm").asString();
        if (rf.check("locate")) locate = rf.find("locate").asString();
        if (rf.check("maxNumBlobs")) maxNumBlobs = rf.find("maxNumBlobs").asInt32();
        if (rf.check("morphClosing")) morphClosing = rf.find("morphClosing").asFloat64();
        if (rf.check("outFeaturesFormat")) outFeaturesFormat = rf.find("outFeaturesFormat").asInt32();

        std::printf("SegmentorThread using algorithm: %s, locate: %s, maxNumBlobs: %d, morphClosing: %f, outFeaturesFormat: %d.\n",
        algorithm.c_str(),locate.c_str(),maxNumBlobs,morphClosing,outFeaturesFormat);*/
    }
    void run(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg);
    //double transform(double value) override;

private:
    double m, b;
    std::string algorithm;

    float area, hue_peak, hue_mode, hue_mean, hue_stddev, saturation_peak,
        saturation_mean, saturation_stddev, value_peak, value_mode, value_mean,
        value_stddev, locX, locY, rectangularity, axisFirst, axisSecond,
        aspectRatio, solidity, massCenterlocX, massCenterlocY, arc, radius;
    double morphClosing;
    std::string locate;
    int maxNumBlobs;
    yarp::os::Bottle outFeatures;
    int outFeaturesFormat;
    int outImage;
    int seeBounding;
    int threshold;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outImageProcessed;
    yarp::os::Bottle outputProcessed;

};

}  // namespace roboticslab

#endif  // __COLOR_REGION_DETECTION_2D_HPP__
