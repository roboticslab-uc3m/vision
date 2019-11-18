// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTION_2D_HPP__
#define __COLOR_REGION_DETECTION_2D_HPP__

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

using namespace yarp::os;
using namespace yarp::sig;

namespace roboticslab
{

class ColorRegionDetection2D {
  private:

  public:
    yarp::sig::ImageOf<yarp::sig::PixelRgb> run(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg);
};

}  // namespace roboticslab

#endif  // __COLOR_REGION_DETECTION_2D_HPP__
