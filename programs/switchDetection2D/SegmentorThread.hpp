// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/FrameGrabberInterfaces.h>

#include <yarp/sig/Image.h>

#include <opencv2/objdetect/objdetect.hpp>

#include <ColorDebug.h>
#include "HaarDetection2D.hpp"
#include "ColorRegionDetection2D.hpp"
#include "TensorflowDetection2D.hpp"
#include "TensorflowDetector.hpp"
#include "Transformation.hpp"
#include <vector>

#define DEFAULT_RATE_MS 20
#define DEFAULT_XMLCASCADE "haarcascade_cocacola_can.xml"
#define DEFAULT_TRAINEDMODEL "frozen_inference_graph.pb"
#define DEFAULT_TRAINEDMODEL_LABELS "labels_map.pbtxt"

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


#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>

#include <yarp/sig/all.h>

#include "cv.h"
//#include "highgui.h" // to show windows

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "TravisLib.hpp"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

using namespace cv;

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
class SegmentorThread : public yarp::os::PeriodicThread
{
private:
    //colorRegion
    std::string algorithm;
    std::string locate;
    int maxNumBlobs;
    double morphClosing;
    Bottle outFeatures;
    int outFeaturesFormat;
    int outImage;
    int seeBounding;
    int threshold;
    float area, hue_peak, hue_mode, hue_mean, hue_stddev, saturation_peak,
        saturation_mean, saturation_stddev, value_peak, value_mode, value_mean,
        value_stddev, locX, locY, rectangularity, axisFirst, axisSecond,
        aspectRatio, solidity, massCenterlocX, massCenterlocY, arc, radius;

    yarp::dev::IFrameGrabberImage *camera;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutImg;  // for testing
    yarp::os::Port *pOutPort;
    yarp::os::BufferedPort<ImageOf<PixelRgb> > outPortShape;

    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *outCropSelectorImg;
    yarp::os::Port *inCropSelectorPort;
    CropSelectorProcessor cropSelectorProcessor;

    std::string strSwitchMode="haarDetection";
    std::string model;
    std::string labels;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;

public:
    Transformation* transformation;

    cv::CascadeClassifier object_cascade;
    SegmentorThread() : PeriodicThread(DEFAULT_RATE_MS * 0.001),
    area(-1), hue_peak(-1), hue_mode(-1), hue_mean(-1), hue_stddev(-1),
    saturation_peak(-1), saturation_mean(-1), saturation_stddev(-1),
    value_peak(-1), value_mode(-1), value_mean(-1), value_stddev(-1),
    locX(-1), locY(-1),
    rectangularity(-1), axisFirst(-1), axisSecond(-1),
    aspectRatio(-1), solidity(-1), massCenterlocX(-1), massCenterlocY(-1),
arc(-1), radius(-1) {}
    void setIFrameGrabberImageDriver(yarp::dev::IFrameGrabberImage * _camera);
    void setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg);
    void setOutPort(yarp::os::Port *_pOutPort);
    void init(yarp::os::ResourceFinder &rf);
    void run();  // The periodical function

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
};

}  // namespace roboticslab

#endif  // __SEGMENTOR_THREAD_HPP__
