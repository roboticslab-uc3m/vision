// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <fstream>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>

#include "OpencvDnnDetector.hpp"

namespace
{
    const std::string DEFAULT_MODEL_FILE = "yolov3-tiny/yolov3-tiny.weights";
    const std::string DEFAULT_CONFIG_DNN_FILE = "yolov3-tiny/yolov3-tiny.cfg";
    const std::string DEFAULT_FRAMEWORK = "darknet";
    const std::string DEFAULT_CLASSES_FILE = "coco-object-categories.txt";
    const int DEFAULT_BACKEND = cv::dnn::DNN_BACKEND_CUDA;
    const int DEFAULT_TARGET = cv::dnn::DNN_TARGET_CUDA;
    const double DEFAULT_SCALE = 0.00392;
    const double DEFAULT_MEAN = 0;
    const double DEFAULT_CONF_THR = 0.1;
    const double DEFAULT_NMS_THR = 0.4;
}

using namespace roboticslab;

bool OpencvDnnDetector::open(yarp::os::Searchable &config)
{
    auto modelFile = config.check("trainedModel", yarp::os::Value(DEFAULT_MODEL_FILE)).asString();
    yDebug() << "Using --trainedModel" << modelFile;

    auto configDNNFile = config.check("configDNNModel", yarp::os::Value(DEFAULT_CONFIG_DNN_FILE)).asString();
    yDebug() << "Using --configDNNModel" << configDNNFile;

    auto framework = config.check("framework", yarp::os::Value(DEFAULT_FRAMEWORK)).asString();
    yDebug() << "Using --framework" << framework;

    auto classesFile = config.check("classesTrainedModel", yarp::os::Value(DEFAULT_CLASSES_FILE)).asString();
    yDebug() << "Using --classesTrainedModel" << classesFile;

    auto backend = config.check("backend", yarp::os::Value(DEFAULT_BACKEND)).asInt32();
    yDebug() << "Using --backend" << backend;

    auto target = config.check("target", yarp::os::Value(DEFAULT_TARGET)).asInt32();
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

    scale = config.check("scale", yarp::os::Value(DEFAULT_SCALE)).asFloat32();
    yDebug() << "Using --scale" << scale;

    auto _mean = config.check("mean", yarp::os::Value(DEFAULT_MEAN)).asFloat64();
    mean = {_mean, _mean, _mean};
    yDebug() << "Using --mean" << _mean;

    confThreshold = config.check("confThreshold", yarp::os::Value(DEFAULT_CONF_THR)).asFloat32();
    yDebug() << "Using --confThreshold" << confThreshold;

    nmsThreshold = config.check("nmsThreshold", yarp::os::Value(DEFAULT_NMS_THR)).asFloat32();
    yDebug() << "Using --nmsThreshold" << nmsThreshold;

    return true;
}

/*****************************************************************/

bool OpencvDnnDetector::detect(const yarp::sig::Image & inYarpImg, yarp::os::Bottle & detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

    // adapted from https://docs.opencv.org/4.3.0/d4/db9/samples_2dnn_2object_detection_8cpp-example.html

    preprocess(inCvMat);

    std::vector<cv::Mat> outs;
    net.forward(outs, outNames);

    postprocess(inCvMat.size(), outs, detectedObjects);

    return true;
}

/************************************************************************/

void OpencvDnnDetector::preprocess(const cv::Mat & frame)
{
    cv::Mat blob;

    // Create a 4D blob from a frame.
    cv::dnn::blobFromImage(frame, blob, 1.0, frame.size(), cv::Scalar(), true, false, CV_8U);

    // Run a model.
    net.setInput(blob, "", scale, mean);

    if (net.getLayer(0)->outputNameToIndex("im_info") != -1) // Faster-RCNN or R-FCN
    {
        cv::Mat imInfo = (cv::Mat_<float>(1, 3) << frame.rows, frame.cols, 1.6f);
        net.setInput(imInfo, "im_info");
    }
}

/************************************************************************/

void OpencvDnnDetector::postprocess(const cv::Size & size, const std::vector<cv::Mat> & outs, yarp::os::Bottle & detectedObjects)
{
    std::vector<int> outLayers = net.getUnconnectedOutLayers();
    std::string outLayerType = net.getLayer(outLayers[0])->type;

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

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
                        left = (int)(data[i + 3] * size.width);
                        top = (int)(data[i + 4] * size.height);
                        right = (int)(data[i + 5] * size.width);
                        bottom = (int)(data[i + 6] * size.height);
                        width = right - left + 1;
                        height = bottom - top + 1;
                    }

                    classIds.push_back((int)(data[i + 1]) - 1); // Skip 0th background class id.
                    boxes.push_back(cv::Rect(left, top, width, height));
                    confidences.push_back(confidence);
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
                    int centerX = (int)(data[0] * size.width);
                    int centerY = (int)(data[1] * size.height);
                    int width = (int)(data[2] * size.width);
                    int height = (int)(data[3] * size.height);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
        }
    }
    else
    {
        yWarning() << "Unknown layer type:" << outLayerType;
        return;
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    for (auto idx : indices)
    {
        const cv::Rect & box = boxes[idx];
        float confidence = confidences[idx];
        const std::string className = classes[classIds[idx]];

        detectedObjects.addDict() = {
            {"category", yarp::os::Value(className)},
            {"confidence", yarp::os::Value(confidence)},
            {"tlx", yarp::os::Value(box.x)},
            {"tly", yarp::os::Value(box.y)},
            {"brx", yarp::os::Value(box.x + box.width)},
            {"bry", yarp::os::Value(box.y + box.height)}
        };

        yInfo() << "Detected" << className << "with" << confidence << "confidence";
    }
}

/************************************************************************/
