// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <fstream>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>

#include "DnnDetector.hpp"

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(DNN, "rl.DnnDetector")
}

bool DnnDetector::open(yarp::os::Searchable &config)
{
    if (!parseParams(config))
    {
        yCError(DNN) << "Failed to parse parameters";
        return false;
    }

    cv::dnn::Backend backend;

    if (m_backend == "default")
    {
        backend = cv::dnn::Backend::DNN_BACKEND_DEFAULT;
    }
    else if (m_backend == "opencv")
    {
        backend = cv::dnn::DNN_BACKEND_OPENCV;
    }
    else if (m_backend == "cuda")
    {
        backend = cv::dnn::DNN_BACKEND_CUDA;
    }
    else
    {
        yCError(DNN) << "Unknown backend:" << m_backend;
        return false;
    }

    cv::dnn::Target target;

    if (m_target == "cpu")
    {
        target = cv::dnn::DNN_TARGET_CPU;
    }
    else if (m_target == "opencl")
    {
        target = cv::dnn::DNN_TARGET_OPENCL;
    }
    else if (m_target == "cuda")
    {
        target = cv::dnn::DNN_TARGET_CUDA;
    }
    else
    {
        yCError(DNN) << "Unknown target:" << m_target;
        return false;
    }

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("DnnDetector");

    // Set model and config path from the resource finder.
    std::string modelPath = rf.findFileByName(m_trainedModel);
    std::string configDNNPath = rf.findFileByName(m_configDNNModel);
    std::string classesPath = rf.findFileByName(m_classesTrainedModel);

    if (configDNNPath.empty())
    {
        yCError(DNN) << "No config dnn file!";
        return false;
    }

    if (modelPath.empty())
    {
        yCError(DNN) << "No model file!";
        return false;
    }

    // Open file with classes names.
    std::ifstream ifs(classesPath.c_str());

    if (!ifs.is_open())
    {
        yCError(DNN) << "Classes file" << classesPath << "not found";
        return false;
    }

    std::string line;

    while (std::getline(ifs, line))
    {
        classes.push_back(std::move(line));
    }

    // Load a model.
    net = cv::dnn::readNet(modelPath, configDNNPath, m_framework);
    net.setPreferableBackend(backend);
    net.setPreferableTarget(target);
    outNames = net.getUnconnectedOutLayersNames();

    mean = {m_mean, m_mean, m_mean};

    return true;
}

/*****************************************************************/

bool DnnDetector::detect(const yarp::sig::Image & inYarpImg, yarp::os::Bottle & detectedObjects)
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

void DnnDetector::preprocess(const cv::Mat & frame)
{
    cv::Mat blob;

    // Create a 4D blob from a frame.
    cv::dnn::blobFromImage(frame, blob, 1.0, frame.size(), cv::Scalar(), true, false, CV_8U);

    // Run a model.
    net.setInput(blob, "", m_scale, mean);

    if (net.getLayer(0)->outputNameToIndex("im_info") != -1) // Faster-RCNN or R-FCN
    {
        cv::Mat imInfo = (cv::Mat_<float>(1, 3) << frame.rows, frame.cols, 1.6f);
        net.setInput(imInfo, "im_info");
    }
}

/************************************************************************/

void DnnDetector::postprocess(const cv::Size & size, const std::vector<cv::Mat> & outs, yarp::os::Bottle & detectedObjects)
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

                if (confidence > m_confThreshold)
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
                    confidences.push_back(confidence);
                    boxes.emplace_back(left, top, width, height);
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

                if (confidence > m_confThreshold)
                {
                    int centerX = (int)(data[0] * size.width);
                    int centerY = (int)(data[1] * size.height);
                    int width = (int)(data[2] * size.width);
                    int height = (int)(data[3] * size.height);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.emplace_back(left, top, width, height);
                }
            }
        }
    }
    else
    {
        yCWarning(DNN) << "Unknown layer type:" << outLayerType;
        return;
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, m_confThreshold, m_nmsThreshold, indices);

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

        yCInfo(DNN) << "Detected" << className << "with" << confidence << "confidence";
    }
}

/************************************************************************/
