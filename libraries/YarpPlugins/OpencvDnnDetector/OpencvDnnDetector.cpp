// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <fstream>
#include <utility>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>

#include "OpencvDnnDetector.hpp"

namespace
{
    const std::string DEFAULT_MODEL_FILE = "yolov3-tiny-custom_5000.weights";
    const std::string DEFAULT_CONFIG_DNN_FILE = "yolov3-tiny-custom.cfg";
    const std::string DEFAULT_FRAMEWORK = "darknet";
    const std::string DEFAULT_CLASSES_FILE = "object_detection_classes_yolov3_sharon.txt";
    const int DEFAULT_BACKEND = cv::dnn::DNN_BACKEND_CUDA;
    const int DEFAULT_TARGET = cv::dnn::DNN_TARGET_CUDA;
    const double DEFAULT_SCALE = 0.00392;
    const unsigned int DEFAULT_WIDTH = 640;
    const unsigned int DEFAULT_HEIGHT = 480;
    const bool DEFAULT_RGB = true;
    const double DEFAULT_MEAN = 0;
    const double DEFAULT_CONF_THR = 0.1;
    const double DEFAULT_NMS_THR = 0.4;
}

using namespace roboticslab;

bool OpencvDnnDetector::open(yarp::os::Searchable &config)
{
    modelFile = config.check("trainedModel", yarp::os::Value(DEFAULT_MODEL_FILE)).asString();

    yDebug() << "Using --trainedModel" << modelFile;

    configDNNFile = config.check("configDNNModel", yarp::os::Value(DEFAULT_CONFIG_DNN_FILE)).asString();

    yDebug() << "Using --configDNNModel" << configDNNFile;

    framework = config.check("framework", yarp::os::Value(DEFAULT_FRAMEWORK)).asString();

    yDebug() << "Using --framework" << framework;

    classesFile = config.check("classesTrainedModel", yarp::os::Value(DEFAULT_CLASSES_FILE)).asString();

    yDebug() << "Using --classesTrainedModel" << classesFile;

    backend = config.check("backend", yarp::os::Value(DEFAULT_BACKEND)).asInt32();

    yDebug() << "Using --backend" << backend;

    target = config.check("target", yarp::os::Value(DEFAULT_TARGET)).asInt32();

    yDebug() << "Using --target" << target;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("OpencvDnnDetector");

    // Set model and config path from the resource finder.
    std::string modelPath = rf.findFileByName(modelFile);
    std::string configDNNPath = rf.findFileByName(configDNNFile);
    std::string classesPath = rf.findFileByName(classesFile);

    if (configDNNPath.empty())
    {
        yError() << "No config dnn file!";
        return false;
    }

    if (modelPath.empty())
    {
        yError() << "No model file!";
        return false;
    }

    // Open file with classes names.
    std::ifstream ifs(classesPath.c_str());

    if (!ifs.is_open())
    {
        yError() << "Classes file" << classesPath << "not found";
        return false;
    }

    std::string line;

    while (std::getline(ifs, line))
    {
        classes.push_back(line);
    }

    // Load a model.
    net = cv::dnn::readNet(modelPath, configDNNPath, framework);
    net.setPreferableBackend(backend);
    net.setPreferableTarget(target);
    outNames = net.getUnconnectedOutLayersNames();

    scale = DEFAULT_SCALE;
    mean = {DEFAULT_MEAN, DEFAULT_MEAN, DEFAULT_MEAN};
    inpWidth = DEFAULT_WIDTH;
    inpHeight = DEFAULT_HEIGHT;
    swapRB = DEFAULT_RGB;
    confThreshold = DEFAULT_CONF_THR;
    nmsThreshold = DEFAULT_NMS_THR;

    return true;
}

/*****************************************************************/

bool OpencvDnnDetector::detect(const yarp::sig::Image &inYarpImg, std::vector<yarp::os::Property> &detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    preprocess(inCvMat, net, cv::Size(inpWidth, inpHeight), scale, mean, swapRB);

    std::vector<cv::Mat> outs;
    net.forward(outs, outNames);

    std::vector<int> outLayers = net.getUnconnectedOutLayers();
    std::string outLayerType = net.getLayer(outLayers[0])->type;

    if (outLayerType == "DetectionOutput")
    {
        // Network produces output blob with a shape 1x1xNx7 where N is a number of
        // detections and an every detection is a vector of values
        // [batchId, classId, confidence, left, top, right, bottom]

        for (const auto & out : outs)
        {
            float *data = (float *)out.data;

            for (std::size_t i = 0; i < out.total(); i += 7)
            {
                float confidence = data[i + 2];

                if (confidence > confThreshold)
                {
                    int left = (int)data[i + 3];
                    int top = (int)data[i + 4];
                    int right = (int)data[i + 5];
                    int bottom = (int)data[i + 6];
                    int width = right - left + 1;
                    int height = bottom - top + 1;

                    if (width <= 2 || height <= 2)
                    {
                        left = (int)(data[i + 3] * inCvMat.cols);
                        top = (int)(data[i + 4] * inCvMat.rows);
                        right = (int)(data[i + 5] * inCvMat.cols);
                        bottom = (int)(data[i + 6] * inCvMat.rows);
                        width = right - left + 1;
                        height = bottom - top + 1;
                    }

                    auto className = classes[(int)(data[i + 1]) - 1];

                    yarp::os::Property detectedObject {
                        {"category", yarp::os::Value(className)},
                        {"confidence", yarp::os::Value(confidence)},
                        {"tlx", yarp::os::Value(left)},
                        {"tly", yarp::os::Value(top)},
                        {"brx", yarp::os::Value(right)},
                        {"bry", yarp::os::Value(bottom)}
                    };

                    yInfo() << "Detected" << className << "with" << confidence << "confidence";
                    detectedObjects.push_back(std::move(detectedObject));
                }
            }
        }
    }
    else if (outLayerType == "Region")
    {
        // Network produces output blob with a shape NxC where N is a number of
        // detected objects and C is a number of classes + 4 where the first 4
        // numbers are [center_x, center_y, width, height]

        for (const auto & out : outs)
        {
            float *data = (float *)out.data;

            for (int i = 0; i < out.rows; ++i, data += out.cols)
            {
                cv::Mat scores = out.row(i).colRange(5, out.cols);
                cv::Point classIdPoint;
                double confidence;
                cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);

                if (confidence > confThreshold)
                {
                    int centerX = (int)(data[0] * inCvMat.cols);
                    int centerY = (int)(data[1] * inCvMat.rows);
                    int width = (int)(data[2] * inCvMat.cols);
                    int height = (int)(data[3] * inCvMat.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    yarp::os::Property detectedObject {
                        {"category", yarp::os::Value(classes[classIdPoint.x])},
                        {"confidence", yarp::os::Value(confidence)},
                        {"tlx", yarp::os::Value(left)},
                        {"tly", yarp::os::Value(top)},
                        {"brx", yarp::os::Value(left + width)},
                        {"bry", yarp::os::Value(top + height)}
                    };

                    yInfo() << "Detected" << classes[classIdPoint.x] << "with" << confidence << "confidence";
                    detectedObjects.push_back(std::move(detectedObject));
                }
            }
        }
    }

    return true;
}

/************************************************************************/

/* Function adapted from https://docs.opencv.org/4.3.0/d4/db9/samples_2dnn_2object_detection_8cpp-example.html */
void OpencvDnnDetector::preprocess(const cv::Mat &frame, cv::dnn::Net &net, cv::Size inpSize, float scale,
                                   const cv::Scalar &mean, bool swapRB)
{
    cv::Mat blob;

    // Create a 4D blob from a frame.
    if (inpSize.width <= 0)
    {
        inpSize.width = frame.cols;
    }

    if (inpSize.height <= 0)
    {
        inpSize.height = frame.rows;
    }

    cv::dnn::blobFromImage(frame, blob, 1.0, inpSize, cv::Scalar(), swapRB, false, CV_8U);

    // Run a model.
    net.setInput(blob, "", scale, mean);

    if (net.getLayer(0)->outputNameToIndex("im_info") != -1) // Faster-RCNN or R-FCN
    {
        cv::resize(frame, frame, inpSize);
        cv::Mat imInfo = (cv::Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
        net.setInput(imInfo, "im_info");
    }
}

/************************************************************************/
