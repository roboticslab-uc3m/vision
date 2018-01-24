// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __VISION_SEGMENTOR_HPP__
#define __VISION_SEGMENTOR_HPP__

#include "SegmentorThread.hpp"

using namespace yarp::os;
using namespace yarp::sig;

class VisionSegmentor : public RFModule {
  private:
    SegmentorThread segmentorThread;
    //
    BufferedPort<ImageOf<PixelRgb> > inImg;
    BufferedPort<ImageOf<PixelRgb> > outImg;
    Port outPort;

    bool interruptModule();
    double getPeriod();
    bool updateModule();

  public:
    bool configure(ResourceFinder &rf);
};

#endif  // __VISION_SEGMENTOR_HPP__

