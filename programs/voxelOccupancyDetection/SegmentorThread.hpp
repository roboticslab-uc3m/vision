// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/all.h>
#include <yarp/dev/IOpenNI2DeviceDriver.h>

#include <yarp/sig/all.h>

// thanks! https://web.stanford.edu/~qianyizh/projects/scenedata.html
#define DEFAULT_FX_D          525.0  // 640x480
#define DEFAULT_FY_D          525.0  //
#define DEFAULT_CX_D          319.5  //
#define DEFAULT_CY_D          239.5  //
#define DEFAULT_FX_RGB        525.0  //
#define DEFAULT_FY_RGB        525.0  //
#define DEFAULT_CX_RGB        319.5  //
#define DEFAULT_CY_RGB        239.5  //

#define DEFAULT_ALGORITHM "blueMinusRed"
#define DEFAULT_LOCATE "centroid"
#define DEFAULT_MAX_NUM_BLOBS 2
#define DEFAULT_MORPH_CLOSING 2
#define DEFAULT_MORPH_OPENING 0
#define DEFAULT_OUT_FEATURES "mmX mmY mmZ"  // it's a bottle!!
#define DEFAULT_OUT_FEATURES_FORMAT 0  // 0=bottled,1=minimal
#define DEFAULT_OUT_IMAGE 1
#define DEFAULT_RATE_MS 20
#define DEFAULT_SEE_BOUNDING 3
#define DEFAULT_THRESHOLD 55

//VoxelOccupancy Constants
#define DEFAULT_SEARCH_AREA_DILATATION 10
#define DEFAULT_AREA_LOW_THRESHOLD 730 //mm; TV Dimensions
#define DEFAULT_AREA_HIGH_THRESHOLD 1250 //mm; TV Dimensions
#define DEFAULT_OCCUPANCY_THRESHOLD 100
#define DEFAULT_CALIBRATION_VALUE_KINECT 0.001923 //This is an approximation (for better results a calibration may be needed)
#define DEFAULT_LOW_Y_BOX_VALUE -15
#define DEFAULT_HIGH_Y_BOX_VALUE 20
//#define DEFAULT_LOW_X_BOX_VALUE -184
//#define DEFAULT_HIGH_X_BOX_VALUE 184
#define DEFAULT_LOW_X_BOX_VALUE -300
#define DEFAULT_HIGH_X_BOX_VALUE 225
#define DEFAULT_VOXEL_RESOLUTION 8 //this is the number of voxel per row.
#define DEFAULT_UTILITY_AREA_LOW_THRESHOLD 1350 //mm;
#define DEFAULT_UTILITY_AREA_HIGH_THRESHOLD 1450 //mm;
#define DEFAULT_NUMBER_UTILITY_VOXELS 4

namespace roboticslab
{

/**
 * @ingroup voxelOccupancyDetection
 *
 * @brief Implements voxelOccupancyDetection callback on Bottle.
 */
class DataProcessor : public yarp::os::PortReader {
    virtual bool read(yarp::os::ConnectionReader& connection) {
        yarp::os::Bottle b;
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

/**
 * @ingroup voxelOccupancyDetection
 *
 * @brief Implements voxelOccupancyDetection RateThread.
 */
class SegmentorThread : public yarp::os::RateThread {
private:
    yarp::dev::IOpenNI2DeviceDriver *kinect;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > *pOutImg;  // for testing
    yarp::os::Port *pOutPort;
    //
    yarp::os::ConstString algorithm;
    yarp::os::ConstString locate;
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
    yarp::os::Bottle outFeatures;
    //
    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg;
    yarp::os::Port* inCropSelectorPort;
    DataProcessor processor;

    //VoxelOccupancy specific variables
    int searchAreaDilatation;
    int areaLowThreshold;
    int areaHighThreshold;
    int occupancyThreshold;
    double kinectCalibrationValue;
    int lowXBox;
    int highXBox;
    int lowYBox;
    int highYBox;
    int voxelResolution;
    int utilityAreaLowThreshold;
    int utilityAreaHighThreshold;
    int numberUtilityVoxels;

public:
    SegmentorThread() : RateThread(DEFAULT_RATE_MS) {}

    void setIKinectDeviceDriver(yarp::dev::IOpenNI2DeviceDriver * _kinect);
    void setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > * _pOutImg);
    void setOutPort(yarp::os::Port *_pOutPort);
    void init(yarp::os::ResourceFinder &rf);
    void run();  // The periodical function

    void setCropSelector(int cropSelector) { this->cropSelector = cropSelector; }
    void setOutCropSelectorImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg) { this->outCropSelectorImg = outCropSelectorImg; }
    void setInCropSelectorPort(yarp::os::Port* inCropSelectorPort) { this->inCropSelectorPort = inCropSelectorPort; }
    
};

}  // namespace roboticslab

#endif  // __SEGMENTOR_THREAD_HPP__

