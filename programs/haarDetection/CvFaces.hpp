// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CV_FACES_HPP__
#define __CV_FACES_HPP__

#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_KINECT_DEVICE "OpenNI2DeviceServer"
#define DEFAULT_KINECT_LOCAL "/cvFaces"
#define DEFAULT_KINECT_REMOTE "/OpenNI2"
#define DEFAULT_WATCHDOG    2       // [s]


namespace teo
{

/**
 * @ingroup cvFaces
 *
 * @brief Computer Vision segment faces.
 */
class CvFaces : public yarp::os::RFModule {
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

}  // namespace teo

#endif  // __CV_FACES_HPP__

