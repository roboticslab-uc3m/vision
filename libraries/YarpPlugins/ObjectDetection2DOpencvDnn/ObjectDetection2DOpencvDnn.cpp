// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Value.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>

#include <fstream>

#include <ColorDebug.h>

#include "ObjectDetection2DOpencvDnn.hpp"
#include "TravisLib.hpp"
namespace roboticslab
{

    /*****************************************************************/

    const std::string ObjectDetection2DOpencvDnn::DEFAULT_MODEL_FILE = "yolov3-tiny-custom_5000.weights";
    const std::string ObjectDetection2DOpencvDnn::DEFAULT_CONFIG_DNN_FILE = "yolov3-tiny-custom.cfg";
    const std::string ObjectDetection2DOpencvDnn::DEFAULT_FRAMEWORK = "darknet";
    const std::string ObjectDetection2DOpencvDnn::DEFAULT_CLASSES_FILE = "object_detection_classes_yolov3_sharon.txt";
    const uint ObjectDetection2DOpencvDnn::DEFAULT_BACKEND = DNN_BACKEND_CUDA;
    const uint ObjectDetection2DOpencvDnn::DEFAULT_TARGET = DNN_TARGET_CUDA;
    const double ObjectDetection2DOpencvDnn::DEFAULT_SCALE = 0.00392;
    const uint ObjectDetection2DOpencvDnn::DEFAULT_WIDTH = 640;
    const uint ObjectDetection2DOpencvDnn::DEFAULT_HEIGHT = 480;
    const bool ObjectDetection2DOpencvDnn::DEFAULT_RGB = true;
    const double ObjectDetection2DOpencvDnn::DEFAULT_MEAN = 0;
    const double ObjectDetection2DOpencvDnn::DEFAULT_CONF_THR = 0.1;
    const double ObjectDetection2DOpencvDnn::DEFAULT_NMS_THR = 0.4;

    /*****************************************************************/

    bool ObjectDetection2DOpencvDnn::open(yarp::os::Searchable &config)
    {
        modelFile = DEFAULT_MODEL_FILE;
        if (config.check("trainedModel"))
        {
            CD_INFO("\"trainedModel\" parameter found\n");
            modelFile = config.find("trainedModel").asString();
        }
        CD_DEBUG("Using \"trainedModel\": %s.\n", modelFile.c_str());

        configDNNFile = DEFAULT_CONFIG_DNN_FILE;

        if (config.check("configDNNModel"))
        {
            CD_INFO("\"configDNNModel\" parameter found\n");
            configDNNFile = config.find("configDNNModel").asString();
        }
        CD_DEBUG("Using \"configDNNModel\": %s.\n", configDNNFile.c_str());

        framework = DEFAULT_FRAMEWORK;

        if (config.check("framework"))
        {
            CD_INFO("\"framework\" parameter found\n");
            framework = config.find("framework").asString();
        }
        CD_DEBUG("Using \"framework\": %s.\n", framework.c_str());

        classesFile = DEFAULT_CLASSES_FILE;

        if (config.check("classesTrainedModel"))
        {
            CD_INFO("\"classesTrainedModel\" parameter found\n");
            classesFile = config.find("classesTrainedModel").asString();
        }
        CD_DEBUG("Using \"classesTrainedModel\": %s.\n", classesFile.c_str());

        backend = DEFAULT_BACKEND;
        if (config.check("backend"))
        {
            CD_INFO("\"backend\" parameter found\n");
            backend = config.find("backend").asInt32();
        }
        CD_DEBUG("Using \"backend\": %d.\n", backend);

        target = DEFAULT_TARGET;

        if (config.check("target"))
        {
            CD_INFO("\"target\" parameter found\n");
            target = config.find("target").asInt32();
        }
        CD_DEBUG("Using \"target\": %d.\n", target);

        yarp::os::ResourceFinder rf;
        rf.setVerbose(false);
        rf.setDefaultContext("ObjectDetection2DOpencvDnn"); //rf.setDefaultContext(context);

        // TODO!! Set model and config path from the resource finder
        std::string modelPath = rf.findFileByName(modelFile);
        std::string configDNNPath = rf.findFileByName(configDNNFile);
        std::string classesPath = rf.findFileByName(classesFile);

        if (configDNNPath.empty())
        {
            yError("No config dnn file!\n");
            std::exit(1);
        }

        if (modelPath.empty())
        {
            yError("No model file!\n");
            std::exit(1);
        }

        // Open file with classes names.
        std::ifstream ifs(classesPath.c_str());
        if (!ifs.is_open())
            yError("File %s not found", classesPath.c_str());
        std::string line;
        while (std::getline(ifs, line))
        {
            classes.push_back(line);
        }

        // Load a model.
        net = readNet(modelPath, configDNNPath, framework);
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

    bool ObjectDetection2DOpencvDnn::detect(const yarp::sig::Image &inYarpImg,
                                            std::vector<yarp::os::Property> &detectedObjects)
    {
        yInfo("Detect");
        yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
        inYarpImgBgr.copy(inYarpImg);
        cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);

        preprocess(inCvMat, net, cv::Size(inpWidth, inpHeight), scale, mean, swapRB);

        std::vector<cv::Mat> outs;
        net.forward(outs, outNames);

        static std::vector<int> outLayers = net.getUnconnectedOutLayers();
        static std::string outLayerType = net.getLayer(outLayers[0])->type;

        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        if (outLayerType == "DetectionOutput")
        {
            yInfo("%s", outLayerType.c_str());

            // Network produces output blob with a shape 1x1xNx7 where N is a number of
            // detections and an every detection is a vector of values
            // [batchId, classId, confidence, left, top, right, bottom]
            CV_Assert(outs.size() > 0);
            for (size_t k = 0; k < outs.size(); k++)
            {
                float *data = (float *)outs[k].data;
                for (size_t i = 0; i < outs[k].total(); i += 7)
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
                        yarp::os::Property detectedObject;
                        detectedObject.put("category", classes[(int)(data[i + 1]) - 1]);
                        detectedObject.put("confidence", confidence);
                        detectedObject.put("tlx", left);
                        detectedObject.put("tly", top);
                        detectedObject.put("brx", right);
                        detectedObject.put("bry", bottom);
                        detectedObjects.push_back(detectedObject);
                    }
                }
            }
        }

        else if (outLayerType == "Region")
        {
            yInfo("%s", outLayerType.c_str());

            for (size_t i = 0; i < outs.size(); ++i)
            {
                // Network produces output blob with a shape NxC where N is a number of
                // detected objects and C is a number of classes + 4 where the first 4
                // numbers are [center_x, center_y, width, height]
                float *data = (float *)outs[i].data;
                for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
                {
                    cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                    cv::Point classIdPoint;
                    double confidence;
                    cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                    if (confidence > confThreshold)
                    {
                        yInfo("%f", confidence);
                        int centerX = (int)(data[0] * inCvMat.cols);
                        int centerY = (int)(data[1] * inCvMat.rows);
                        int width = (int)(data[2] * inCvMat.cols);
                        int height = (int)(data[3] * inCvMat.rows);
                        int left = centerX - width / 2;
                        int top = centerY - height / 2;

                        yarp::os::Property detectedObject;
                        detectedObject.put("tlx", left);
                        detectedObject.put("tly", top);
                        detectedObject.put("brx", left + width);
                        detectedObject.put("bry", top + height);
                        detectedObject.put("category", classes[classIdPoint.x]);
                        detectedObject.put("confidence", confidence);
                        yInfo("Detected %s with %f confidence", classes[classIdPoint.x].c_str(), confidence);
                        detectedObjects.push_back(detectedObject);
                    }
                }
            }
        }


        return true;
    }
    /************************************************************************/
    /* Function adapted from https://docs.opencv.org/4.3.0/d4/db9/samples_2dnn_2object_detection_8cpp-example.html */
    inline void ObjectDetection2DOpencvDnn::preprocess(const cv::Mat &frame, Net &net, cv::Size inpSize, float scale,
                                                       const cv::Scalar &mean, bool swapRB)
    {
        static cv::Mat blob;
        // Create a 4D blob from a frame.
        if (inpSize.width <= 0)
            inpSize.width = frame.cols;
        if (inpSize.height <= 0)
            inpSize.height = frame.rows;
        blobFromImage(frame, blob, 1.0, inpSize, cv::Scalar(), swapRB, false, CV_8U);

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

} // namespace roboticslab
