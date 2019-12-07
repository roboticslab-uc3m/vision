// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DETECTOR_THREAD_HPP__
#define __DETECTOR_THREAD_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/FrameGrabberInterfaces.h>

#include <yarp/sig/Image.h>

#include <ColorDebug.h>

#include "Detector.hpp"

#define DEFAULT_RATE_MS 20

namespace roboticslab
{

/**
 * @ingroup switchDetection2D
 *
 * @brief Implements switchDetection2D callback on Bottle.
 */
class CropSelectorProcessor : public yarp::os::PortReader
{
private:
    virtual bool read(yarp::os::ConnectionReader& connection)
    {
        yarp::os::Bottle b;
        b.read(connection);

        // process data in b
        CD_DEBUG("Got %s\n", b.toString().c_str());

        if (waitForFirst)
        {
            xKeep = b.get(0).asInt32();
            yKeep = b.get(1).asInt32();
            waitForFirst = false;
        }
        else
        {
            if (b.get(0).asInt32() < xKeep || b.get(1).asInt32() < yKeep)
            {
                x = y = w = h = 0;
            }
            else
            {
                x = y = xKeep;
                w = b.get(0).asInt32() - x;
                h = b.get(1).asInt32() - y;
            }

            waitForFirst = true;
        }

        return true;
    }

public:
    bool reset()
    {
        waitForFirst = true;
        x = y = w = h = 0;
        xKeep = yKeep = 0;
        return true;
    }

    int xKeep, yKeep;
    int x, y, w, h;

    bool waitForFirst;

};

/**
 * @ingroup switchDetection2D
 *
 * @brief Implements switchDetection2D PeriodicThread.
 */
class DetectorThread : public yarp::os::PeriodicThread
{
public:
    DetectorThread() : PeriodicThread(DEFAULT_RATE_MS * 0.001) {}

    void setIFrameGrabberImageDriver(yarp::dev::IFrameGrabberImage * _camera);
    void setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg);
    void setOutPort(yarp::os::Port *_pOutPort);
    bool init(yarp::os::ResourceFinder &rf);

    void setCropSelector(int cropSelector)
    {
        this->cropSelector = cropSelector;
    }

    void setOutCropSelectorImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg)
    {
        this->outCropSelectorImg = outCropSelectorImg;
    }

    void setInCropSelectorPort(yarp::os::Port* inCropSelectorPort)
    {
        this->inCropSelectorPort = inCropSelectorPort;
    }

private:
    void run() override;  // The periodical function

    Detector* detector;

    yarp::dev::IFrameGrabberImage *camera;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutImg;  // for testing
    yarp::os::Port *pOutPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPortShape;

    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *outCropSelectorImg;
    yarp::os::Port *inCropSelectorPort;
    CropSelectorProcessor cropSelectorProcessor;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
};

}  // namespace roboticslab

#endif  // __DETECTOR_THREAD_HPP__
