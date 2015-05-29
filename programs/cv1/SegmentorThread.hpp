// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/all.h>
#include <yarp/dev/IOpenNI2DeviceDriver.h>

#include <yarp/sig/all.h>

#include <yarp/math/Math.h>

#include "cv.h"
//#include "highgui.h" // to show windows

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "TravisLib.hpp"
#include "TinyMath.hpp"

// thanks! http://nicolas.burrus.name/index.php/Research/KinectCalibration
#define DEFAULT_FX_D          594.21434211923247  // 640
#define DEFAULT_FY_D          591.04053696870778  // 640
#define DEFAULT_CX_D          339.30780975300314  // 320
#define DEFAULT_CY_D          242.73913761751615  // 240
#define DEFAULT_FX_RGB        529.21508098293293  // 640
#define DEFAULT_FY_RGB        525.56393630057437  // 640
#define DEFAULT_CX_RGB        328.94272028759258  // 320
#define DEFAULT_CY_RGB        267.48068171871557  // 240

#define DEFAULT_HEIGHT         512     // Base TransZ [mm]
#define DEFAULT_PAN            0     // Base RotZ [deg]
#define DEFAULT_TILT           -42     // Extra RotY [deg], neg looks down as in OpenNI, -42 goes good with real -50

#define DEFAULT_ALGORITHM "blueMinusRed"
#define DEFAULT_LOCATE "centroid"
#define DEFAULT_MAX_NUM_BLOBS 2
#define DEFAULT_MORPH_CLOSING 2
#define DEFAULT_MORPH_OPENING 0
#define DEFAULT_OUT_FEATURES "locX0 locY0 locZ0 area hue sat val angle aspectRatio rectangularity axisFirst axisSecond solidity hueStdDev satStdDev valStdDev"  // it's a bottle!!
#define DEFAULT_OUT_FEATURES_FORMAT 0  // 0=bottled,1=minimal
#define DEFAULT_OUT_IMAGE 1
#define DEFAULT_RATE_MS 20
#define DEFAULT_SEE_BOUNDING 3
#define DEFAULT_THRESHOLD 55

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::sig::draw;

class DataProcessor : public PortReader {
    virtual bool read(ConnectionReader& connection) {
        Bottle b;
        b.read(connection);
        // process data in b
        printf("Got %s\n", b.toString().c_str());
        if(waitForFirst) {
            xKeep = b.get(0).asInt();
            yKeep = b.get(1).asInt();
            waitForFirst = false;
        } else {
            if((b.get(0).asInt()<xKeep)||(b.get(1).asInt()<yKeep)){
                x = 0;
                y = 0;
                w = 0;
                h = 0;
            } else {
                x = xKeep;
                y = yKeep;
                w = b.get(0).asInt() - x;
                h = b.get(1).asInt() - y;
            }
            waitForFirst = true;
        }
        return true;

    }
public:
    bool reset() {
        waitForFirst = true;
        x = 0;
        y = 0;
        w = 0;
        h = 0;
        xKeep = 0;
        yKeep = 0;
    }
    int xKeep, yKeep;
    int x, y, w, h;
    bool waitForFirst;
};

class SegmentorThread : public RateThread {
private:
    IOpenNI2DeviceDriver *kinect;
    BufferedPort<ImageOf<PixelRgb> > *pOutImg;  // for testing
    Port *pOutPort;
    //
    ConstString algorithm;
    ConstString locate;
    int maxNumBlobs;
    double morphClosing;
    double morphOpening;
    int outFeaturesFormat;
    int outImage;
    int seeBounding;
    int threshold;
    //
    double fx_d,fy_d,cx_d,cy_d,fx_rgb,fy_rgb,cx_rgb,cy_rgb;
    //
    double height, pan, tilt;
    //
    Bottle outFeatures;
    //
    yarp::sig::Matrix H_0_k;
    //
    int cropSelector;
    BufferedPort<ImageOf<PixelRgb> >* outCropSelectorImg;
    Port* inCropSelectorPort;
    DataProcessor processor;

public:
    SegmentorThread() : RateThread(DEFAULT_RATE_MS) {}

    void setIKinectDeviceDriver(IOpenNI2DeviceDriver * _kinect);
    void setOutImg(BufferedPort<ImageOf<PixelRgb> > * _pOutImg);
    void setOutPort(Port *_pOutPort);
    void init(ResourceFinder &rf);
    void run();  // The periodical function

    void setCropSelector(int cropSelector) { this->cropSelector = cropSelector; }
    void setOutCropSelectorImg(BufferedPort<ImageOf<PixelRgb> >* outCropSelectorImg) { this->outCropSelectorImg = outCropSelectorImg; }
    void setInCropSelectorPort(Port* inCropSelectorPort) { this->inCropSelectorPort = inCropSelectorPort; }
    
};

#endif  // __SEGMENTOR_THREAD_HPP__

