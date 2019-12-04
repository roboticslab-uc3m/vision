// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTION_2D_HPP__
#define __COLOR_REGION_DETECTION_2D_HPP__

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv.h"
#include "highgui.h" // to show windows

#include <ColorDebug.h>
#include "SegmentorThread.hpp"
#include "TravisLib.hpp"

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

class ColorRegionDetection2D
{
public:
    void run(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg, std::string algorithm, std::string locate, double morphClosing, int maxNumBlobs, int threshold);
    yarp::os::Bottle outFeatures;
    int outFeaturesFormat;
    int outImage;
    int seeBounding;

private:
    float area, hue_peak, hue_mode, hue_mean, hue_stddev,
        saturation_peak, saturation_mean, saturation_stddev,
        value_peak, value_mode, value_mean, value_stddev, locX, locY,
        rectangularity, axisFirst, axisSecond, aspectRatio, solidity, massCenterlocX, massCenterlocY, arc, radius;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outImageProcessed;
    yarp::os::Bottle outputProcessed;
};

}  // namespace roboticslab

#endif  // __COLOR_REGION_DETECTION_2D_HPP__
