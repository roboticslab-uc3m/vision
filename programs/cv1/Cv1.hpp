// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CV1_HPP__
#define __CV1_HPP__

#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_KINECT_DEVICE "KinectDeviceLocal"
#define DEFAULT_KINECT_LOCAL "/visionDepth"
#define DEFAULT_KINECT_REMOTE "/kinect"
#define DEFAULT_WATCHDOG    2       // [s]

using namespace yarp::os;
using namespace yarp::sig;

class Cv1 : public RFModule {
  private:
    SegmentorThread segmentorThread;
    //
    PolyDriver dd;
    IKinectDeviceDriver *kinect;

    BufferedPort<ImageOf<PixelRgb> > outImg;
    Port outPort;

    int cropSelector;
    BufferedPort<ImageOf<PixelRgb> > outCropSelectorImg;
    Port inCropSelectorPort;

    bool interruptModule();
    double getPeriod();
    bool updateModule();
    double watchdog;

  public:
    bool configure(ResourceFinder &rf);
};

#endif  // __CV1_HPP__

