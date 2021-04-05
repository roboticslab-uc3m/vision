// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/conf/version.h>
#include <yarp/os/Time.h>

#include "SegmentorThread.hpp"


namespace roboticslab
{

/************************************************************************/

void SegmentorThread::setIRGBDSensor(yarp::dev::IRGBDSensor *_iRGBDSensor)
{
    iRGBDSensor = _iRGBDSensor;
}

/************************************************************************/

void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/

void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/

bool SegmentorThread::init(yarp::os::ResourceFinder &rf)
{
    int rateMs = DEFAULT_RATE_MS;

    yarp::os::Property depthIntrinsicParams;

    if(!iRGBDSensor->getDepthIntrinsicParam(depthIntrinsicParams))
    {
        yError("No intrinsic params");
        return false;
    }

    fx_d = depthIntrinsicParams.find("focalLengthX").asFloat64();
    fy_d = depthIntrinsicParams.find("focalLengthY").asFloat64();
    cx_d = depthIntrinsicParams.find("principalPointX").asFloat64();
    cy_d = depthIntrinsicParams.find("principalPointY").asFloat64();



    confThreshold = DEFAULT_CONF_THR;
    nmsThreshold = DEFAULT_NMS_THR;
    scale = DEFAULT_SCALE;
    mean = {0,0,0};
    swapRB = DEFAULT_RGB;
    inpWidth = DEFAULT_WIDTH;
    inpHeight = DEFAULT_HEIGHT;
    modelFile = DEFAULT_MODEL_NAME;
    configDNNFile = DEFAULT_DNN_CONFIG_FILE;
    framework = DEFAULT_FRAMEWORK;
    classesFile = DEFAULT_CLASSES;



    yInfo("SegmentorThread options:\n");
    yInfo("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    yInfo("\t--rateMs (default: \"%d\")\n",rateMs);
    yInfo("\t--framework (default: \"%s\")\n", framework.c_str());
    yInfo("\t--configDNNFile (default: \"%s\")\n", configDNNFile.c_str());
    yInfo("\t--modelFile (default: \"%s\")\n", modelFile.c_str());
    yInfo("\t--inpWidth (default: \"%d\")\n",inpWidth);
    yInfo("\t--inpHeight (default: \"%d\")\n",inpHeight);
    yInfo("\t--mean (default: \"%d\")\n",mean[0]);
    yInfo("\t--scale (default: \"%f\")\n",scale);
    yInfo("\t--confThreshold (default: \"%f\")\n",confThreshold);
    yInfo("\t--nmsThreshold (default: \"%f\")\n",nmsThreshold);



    yInfo("SegmentorThread using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n", fx_d,fy_d,cx_d,cy_d);

    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt32();
    }

    if(rf.check("framework")){
        framework = rf.find("framework").asString();
        yInfo("Selected framework from resource finder %s",framework.c_str());
    }

    if(rf.check("configDNNModel")){
        configDNNFile = rf.find("configDNNModel").asString();
        yInfo("Selected configDNNModel from resource finder %s",configDNNFile.c_str());
    }


    if(rf.check("trainedModel")){
        modelFile = rf.find("trainedModel").asString();
        yInfo("Selected trainedModel from resource finder %s",modelFile.c_str());
    }

    if(rf.check("classesTrainedModel")){
        classesFile = rf.find("classesTrainedModel").asString();
        yInfo("Selected classesTrainedModel from resource finder %s",classesFile.c_str());
    }



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
        yError("File %s not found",classesPath.c_str());
    std::string line;
    while (std::getline(ifs, line))
    {
        classes.push_back(line);
    }

    // Load a model.
    net = readNet(modelPath, configDNNPath, framework);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    outNames = net.getUnconnectedOutLayersNames();

    if(cropSelector != 0)
    {
        processor.reset();
        inCropSelectorPort->setReader(processor);
    }

#if YARP_VERSION_MINOR < 5
    // Wait for the first few frames to arrive. We kept receiving invalid pixel codes
    // from the depthCamera device if started straight away.
    // https://github.com/roboticslab-uc3m/vision/issues/88
    yarp::os::Time::delay(1);
#endif

    if(!setPeriod(rateMs * 0.001))
    {
        yError("\n");
        return false;
    }

    if(!start())
    {
        yError("\n");
        return false;
    }
    return true;
}


/************************************************************************/
/* Function adapted from https://docs.opencv.org/4.3.0/d4/db9/samples_2dnn_2object_detection_8cpp-example.html */
inline void SegmentorThread::preprocess(const cv::Mat& frame, Net& net, cv::Size inpSize, float scale,
                       const cv::Scalar& mean, bool swapRB)
{
    static cv::Mat blob;
    // Create a 4D blob from a frame.
    if (inpSize.width <= 0) inpSize.width = frame.cols;
    if (inpSize.height <= 0) inpSize.height = frame.rows;
    blobFromImage(frame, blob, 1.0, inpSize, cv::Scalar(), swapRB, false, CV_8U);

    // Run a model.
    net.setInput(blob, "", scale, mean);
    if (net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
    {
        cv::resize(frame, frame, inpSize);
        cv::Mat imInfo = (cv::Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
        net.setInput(imInfo, "im_info");
    }
}
/************************************************************************/
/* Function adapted from https://docs.opencv.org/4.3.0/d4/db9/samples_2dnn_2object_detection_8cpp-example.html */
void SegmentorThread::postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, Net& net)
{
    static std::vector<int> outLayers = net.getUnconnectedOutLayers();
    static std::string outLayerType = net.getLayer(outLayers[0])->type;

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    yarp::os::Bottle output;
    output.clear();

    yInfo()<<outLayerType;

    if (outLayerType == "DetectionOutput")
    {
        // Network produces output blob with a shape 1x1xNx7 where N is a number of
        // detections and an every detection is a vector of values
        // [batchId, classId, confidence, left, top, right, bottom]
        CV_Assert(outs.size() > 0);
        for (size_t k = 0; k < outs.size(); k++)
        {
            float* data = (float*)outs[k].data;
            for (size_t i = 0; i < outs[k].total(); i += 7)
            {
                float confidence = data[i + 2];
                if (confidence > confThreshold)
                {
                    int left   = (int)data[i + 3];
                    int top    = (int)data[i + 4];
                    int right  = (int)data[i + 5];
                    int bottom = (int)data[i + 6];
                    int width  = right - left + 1;
                    int height = bottom - top + 1;
                    if (width <= 2 || height <= 2)
                    {
                        left   = (int)(data[i + 3] * frame.cols);
                        top    = (int)(data[i + 4] * frame.rows);
                        right  = (int)(data[i + 5] * frame.cols);
                        bottom = (int)(data[i + 6] * frame.rows);
                        width  = right - left + 1;
                        height = bottom - top + 1;
                    }
                    yarp::os::Property detectedObject;
                    detectedObject.put("category", classes[(int)(data[i+1])-1]);
                    detectedObject.put("confidence", confidence);
                    detectedObject.put("tlx", left);
                    detectedObject.put("tly", top);
                    detectedObject.put("brx", right);
                    detectedObject.put("bry", bottom);
                    output.addDict()=detectedObject;
                    classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
                    boxes.push_back(cv::Rect(left, top, width, height));
                    confidences.push_back(confidence);
                }
            }
        }
    }
    else if (outLayerType == "Region")
    {
        for (size_t i = 0; i < outs.size(); ++i)
        {
            // Network produces output blob with a shape NxC where N is a number of
            // detected objects and C is a number of classes + 4 where the first 4
            // numbers are [center_x, center_y, width, height]
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                cv::Point classIdPoint;
                double confidence;
                cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);

                if (confidence > confThreshold)
                {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(cv::Rect(left, top, width, height));

                    yarp::os::Property detectedObject;
                    detectedObject.clear();
                    detectedObject.put("tlx", left);
                    detectedObject.put("tly", top);
                    detectedObject.put("brx", left+width);
                    detectedObject.put("bry", top+height);
                    detectedObject.put("category",classes[classIdPoint.x]);
                    detectedObject.put("confidence", confidence);
                    yInfo("Detected %s with %f confidence",classes[classIdPoint.x].c_str(), confidence);
                    output.addDict()=detectedObject;
                }
            }
        }
    }
    else
        CV_Error(cv::Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);

    std::vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
    
    if (output.size() > 0)
    {
        pOutPort->write(output);
    }
}
/************************************************************************/

void SegmentorThread::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0));

    std::string label = cv::format("%.3f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ": " + label;
    }

    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    top = cv::max(top, labelSize.height);
    cv::rectangle(frame, cv::Point(left, top - labelSize.height),
              cv::Point(left + labelSize.width, top + baseLine), cv::Scalar::all(255), cv::FILLED);
    cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar());
}

/************************************************************************/
void SegmentorThread::run()
{
    yarp::sig::FlexImage colorFrame;

    if (!iRGBDSensor->getRgbImage(colorFrame,NULL))
    {
        return;
    }
    yarp::sig::ImageOf<yarp::sig::PixelBgr> colorFrameBgr;
    colorFrameBgr.copy(colorFrame);
    cv::Mat frame=yarp::cv::toCvMat(colorFrameBgr);


    preprocess(frame, net, cv::Size(inpWidth, inpHeight), scale, mean, swapRB);

    std::vector<cv::Mat> outs;
    net.forward(outs, outNames);

    postprocess(frame, outs, net);


    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    outYarpImg.setExternal(frame.data, frame.size[1], frame.size[0]);
    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

}

}  // namespace roboticslab
