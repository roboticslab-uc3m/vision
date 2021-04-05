// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Property.h>

#include <yarp/dev/all.h>
#include <yarp/dev/IRGBDSensor.h>

#include <yarp/sig/all.h>

#include <yarp/cv/Cv.h>


#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/text.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <sstream>


#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <fstream>

#define DEFAULT_RATE_MS 30
#define DEFAULT_ALIAS "yolo" // An alias name of model to extract preprocessing parameters from models.yml file.
#define DEFAULT_ZOO_FILE "models.yml" // An optional path to file with preprocessing parameters models.yml
#define DEFAULT_CLASSES "object_detection_classes_yolov3.txt" // Optional path to a text file with names of classes to label detected objects.
#define DEFAULT_DNN_CONFIG_FILE "yolov3.cfg"
#define DEFAULT_MODEL_NAME "yolov3.weights"
#define DEFAULT_MEAN "0 0 0"
#define DEFAULT_SCALE 0.00392
#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480
#define DEFAULT_RGB true
#define DEFAULT_CONF_THR 0.1 // Confidence threshold.
#define DEFAULT_NMS_THR 0.4 // Non-maximum suppression threshold.
#define DEFAULT_BACKEND cv::dnn::DNN_BACKEND_CUDA
#define DEFAULT_TARGET cv::dnn::DNN_TARGET_CUDA
#define DEFAULT_FRAMEWORK "darknet"



//using namespace cv;
using namespace cv::dnn;
namespace roboticslab
{

/**
 * @ingroup opencvDnnObjectDetection2D
 *
 * @brief Implements opencvDnnObjectDetection2D callback on Bottle.
 */
class DataProcessor : public yarp::os::PortReader
{
    virtual bool read(yarp::os::ConnectionReader& connection)
    {
        yarp::os::Bottle b;
        b.read(connection);
        // process data in b
        printf("Got %s\n", b.toString().c_str());

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
                x = 0;
                y = 0;
                w = 0;
                h = 0;
            }
            else
            {
                x = xKeep;
                y = yKeep;
                w = b.get(0).asInt32() - x;
                h = b.get(1).asInt32() - y;
            }

            waitForFirst = true;
        }

        return true;
    }

public:
    void reset()
    {
        waitForFirst = true;
        x = y = w = h = 0;
        xKeep = yKeep = 0;
    }

    int xKeep, yKeep;
    int x, y, w, h;
    bool waitForFirst;
};

/**
 * @ingroup opencvDnnObjectDetection2D
 *
 * @brief Implements opencvDnnObjectDetection2D PeriodicThread.
 */
class SegmentorThread : public yarp::os::PeriodicThread
{
public:
    SegmentorThread() : PeriodicThread(DEFAULT_RATE_MS * 0.001) {}

    void setIRGBDSensor(yarp::dev::IRGBDSensor * _iRGBDSensor);
    void setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg);
    void setOutPort(yarp::os::Port *_pOutPort);
    bool init(yarp::os::ResourceFinder &rf);

    void setCropSelector(int cropSelector) { this->cropSelector = cropSelector; }
    void setOutCropSelectorImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg) { this->outCropSelectorImg = outCropSelectorImg; }
    void setInCropSelectorPort(yarp::os::Port* inCropSelectorPort) { this->inCropSelectorPort = inCropSelectorPort; }

    inline void preprocess(const cv::Mat& frame, Net& net, cv::Size inpSize, float scale, const cv::Scalar& mean, bool swapRB);
    void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, Net& net);
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

private:
    void run() override; // The periodical function


    yarp::dev::IRGBDSensor *iRGBDSensor;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutImg;  // for testing
    yarp::os::Port *pOutPort;
    //
    double fx_d,fy_d,cx_d,cy_d;
    //
    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg;
    yarp::os::Port* inCropSelectorPort;
    DataProcessor processor;

    // Opencv Dnn
    float confThreshold; // Confidence threshold
    float nmsThreshold; // Non-max supression threshold
    std::vector<std::string> classes;

    std::string modelName; // Not used
    std::string zooFile; //Not used
    float scale;
    cv::Scalar mean;
    bool swapRB;
    int inpWidth;
    int inpHeight;
    std::string modelFile;
    std::string configDNNFile;
    std::string framework;
    std::string classesFile;
    Net net;
    std::vector<std::string> outNames;
};

}  // namespace roboticslab

#endif  // __SEGMENTOR_THREAD_HPP__
