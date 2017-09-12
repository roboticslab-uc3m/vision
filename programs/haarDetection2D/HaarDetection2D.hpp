// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTION_2D_HPP__
#define __HAAR_DETECTION_2D_HPP__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#include <yarp/sig/Image.h>

#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_CAMERA_DEVICE "remote_grabber"
#define DEFAULT_CAMERA_LOCAL "/haarDetection2D"
#define DEFAULT_CAMERA_REMOTE "/frameGrabber2D"
#define DEFAULT_WATCHDOG    2       // [s]

namespace roboticslab
{

/**
 * @ingroup haarDetection2D
 *
 * @brief Computer Vision segment faces.
 */
class HaarDetection2D : public yarp::os::RFModule
{
private:
    SegmentorThread segmentorThread;

    yarp::dev::PolyDriver dd;
    yarp::dev::IFrameGrabberImage *camera;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outImg;
    yarp::os::Port outPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outCropSelectorImg;
    yarp::os::Port inCropSelectorPort;

    int cropSelector;
    double watchdog;

    bool interruptModule();
    double getPeriod();
    bool updateModule();
    bool close();

public:
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace roboticslab

#endif  // __HAAR_DETECTION_2D_HPP__
