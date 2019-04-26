// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/sig/Image.h>
#include <ColorDebug.h>
#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <time.h>
#include <cstdlib>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.hpp>
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"
#include "TensorflowDetector.hpp"
#include <map>
#include <string.h>

// http://web.archive.org/web/20150524152748/https://web.stanford.edu/~qianyizh/projects/scenedata.html
#define DEFAULT_FX_D          525.0  // 640x480
#define DEFAULT_FY_D          525.0  //
#define DEFAULT_CX_D          319.5  //
#define DEFAULT_CY_D          239.5  //

#define DEFAULT_RATE_MS 100
#define DEFAULT_TRAINEDMODEL "frozen_inference_graph.pb"
#define DEFAULT_TRAINEDMODEL_LABELS "labels_map.pbtxt"

namespace roboticslab
{

/**
 * @ingroup tensorflowDetection2D
 *
 * @brief Implements tensorflowDetection2D callback on Bottle.
 */
class DataProcessor : public yarp::os::PortReader
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
            xKeep = b.get(0).asInt();
            yKeep = b.get(1).asInt();
            waitForFirst = false;
        }
        else
        {
            if (b.get(0).asInt() < xKeep || b.get(1).asInt() < yKeep)
            {
                x = y = w = h = 0;
            }
            else
            {
                x = y = xKeep;
                w = b.get(0).asInt() - x;
                h = b.get(1).asInt() - y;
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
 * @ingroup tensorflowDetection2D
 *
 * @brief Implements tensorflowDetection2D RateThread.
 */
class SegmentorThread : public yarp::os::RateThread
{
private:
    yarp::dev::IFrameGrabberImage *camera;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutImg;  // for testing
    yarp::os::Port *pOutPort;

    double fx_d, fy_d, cx_d, cy_d;
    int cropSelector;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *outCropSelectorImg;
    yarp::os::Port *inCropSelectorPort;
    yarp::os::Port resultsPort;
    yarp::os::Bottle bottle;
    DataProcessor processor;


public:
    SegmentorThread() : RateThread(DEFAULT_RATE_MS) {}

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

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;
    tensorflow::Status readLabelsMapStatus;
    tensorflow::Status loadGraphStatus;
    tensorflow::string inputLayer;
    tensorflow::TensorShape shape;
    tensorflow::Tensor tensor;
    std::vector<tensorflow::Tensor> outputs;
    std::unique_ptr<tensorflow::Session> session;
    std::map<int, std::string> labelsMap;
    std::vector<std::string> outputLayer;
    std::string model;
    std::string labels;
    time_t start, end;
    int nFrames = 25;
    int iFrame = 0;
    double fps = 0.;
    double thresholdScore = 0.5;
    double thresholdIOU = 0.8;

};

}  // namespace roboticslab

#endif  // __SEGMENTOR_THREAD_HPP__
