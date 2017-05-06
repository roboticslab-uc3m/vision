// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTION_HPP__
#define __HAAR_DETECTION_HPP__

#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_KINECT_DEVICE "OpenNI2DeviceServer"
#define DEFAULT_KINECT_LOCAL "/haarDetection"
#define DEFAULT_KINECT_REMOTE "/OpenNI2"
#define DEFAULT_WATCHDOG    2       // [s]


namespace roboticslab
{

/**
 * @ingroup haarDetection
 *
 * @brief Computer Vision segment faces.
 */
class HaarDetection : public yarp::os::RFModule {
  private:
    SegmentorThread segmentorThread;
    //
    yarp::dev::PolyDriver dd;
    yarp::dev::IOpenNI2DeviceDriver *kinect;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outImg;
    yarp::os::Port outPort;

    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outCropSelectorImg;
    yarp::os::Port inCropSelectorPort;

    bool interruptModule();
    double getPeriod();
    bool updateModule();
    double watchdog;

  public:
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace roboticslab

#endif  // __HAAR_DETECTION_HPP__

