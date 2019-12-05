// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SWITCH_DETECTION_2D_HPP__
#define __SWITCH_DETECTION_2D_HPP__

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>

#include "SegmentorThread.hpp"

namespace roboticslab
{
/**
 * @ingroup switchDetection2D
 *
 * @brief Computer Vision segment faces.
 */
class SwitchDetection2D : public yarp::os::RFModule
{
private:
    SegmentorThread segmentorThread;

    yarp::dev::PolyDriver cameraDevice;
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

#endif  // __SWITCH_DETECTION_2D_HPP__
