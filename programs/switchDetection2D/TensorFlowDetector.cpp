// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <fstream>
#include <regex>

#include <yarp/os/ResourceFinder.h>

#include <yarp/sig/ImageDraw.h>

#include <cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tensorflow/cc/ops/standard_ops.h>

#include <ColorDebug.h>

#include "TensorFlowDetector.hpp"

namespace roboticslab
{

/*****************************************************************/

const std::string TensorFlowDetector::DEFAULT_TRAINEDMODEL = "frozen_inference_graph.pb";
const std::string TensorFlowDetector::DEFAULT_TRAINEDMODEL_LABELS = "labels_map.pbtxt";
const double TensorFlowDetector::DEFAULT_THRESHOLD_SCORE = 0.5;
const double TensorFlowDetector::DEFAULT_THRESHOLD_IOU = 0.8;

/*****************************************************************/

TensorFlowDetector::TensorFlowDetector(yarp::os::Searchable* parameters) : firstArrived(false)
{
    std::string trainedModel = DEFAULT_TRAINEDMODEL;
    CD_DEBUG("\"trainedModel\" [file.pb] (default: \"%s\")\n", trainedModel.c_str());
    if(parameters->check("trainedModel"))
    {
        CD_INFO("\"trainedModel\" parameter for TensorFlowDetector found\n");
        trainedModel = parameters->find("trainedModel").asString();
    }
    CD_DEBUG("\"trainedModel\" parameter for TensorFlowDetector found: \"%s\"\n", trainedModel.c_str());

    std::string trainedModelLabels = DEFAULT_TRAINEDMODEL_LABELS;
    CD_DEBUG("\"trainedModelLabels\" [file.pbtxt] (default: \"%s\")\n", trainedModelLabels.c_str());
    if(parameters->check("trainedModelLabels"))
    {
        CD_INFO("\"trainedModelLabels\" parameter for TensorFlowDetector found\n");
        trainedModelLabels = parameters->find("trainedModelLabels").asString();
    }
    CD_DEBUG("\"trainedModelLabels\" parameter for TensorFlowDetector found: \"%s\"\n", trainedModelLabels.c_str());

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("switchDetection2D");

    std::string trainedModelFullName = rf.findFileByName(trainedModel);
    if(trainedModelFullName.empty())
    {
        CD_ERROR("full path for \"trainedModel\" NOT found\n");
        return;
    }
    CD_DEBUG("full path for \"trainedModel\" found: \"%s\"\n", trainedModelFullName.c_str());

    std::string trainedModelLabelsFullName = rf.findFileByName(trainedModelLabels);
    if(trainedModelLabelsFullName.empty())
    {
        CD_ERROR("full path for \"trainedModelLabels\" NOT found\n");
        return;
    }
    CD_DEBUG("full path for \"trainedModelLabels\" found: \"%s\"\n", trainedModelLabelsFullName.c_str());

    // Set  node names
    inputLayer = "image_tensor:0";
    outputLayer = {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"};

    // Load .pb frozen model
    tensorflow::string graphPath = trainedModelFullName; // GRAPH
    tensorflow::Status loadGraphStatus = loadGraph(graphPath, &session);
    if (!loadGraphStatus.ok())
    {
        CD_ERROR("Fail loading graph \"%s\"\n",graphPath.c_str());
        return;
    }
    CD_SUCCESS("Graph \"%s\" loaded correctly\n",graphPath.c_str());

    // Load labels
    CD_INFO("Labels \"%s\" are going to be loaded.\n",trainedModelLabelsFullName.c_str());
    tensorflow::Status readLabelsMapStatus = readLabelsMapFile(trainedModelLabelsFullName, labelsMap);
    if (!readLabelsMapStatus.ok())
    {
        CD_ERROR("Fail loading labels \"%s\".\n",trainedModelLabelsFullName.c_str());
        return;
    }
    CD_SUCCESS("Labels \"%s\" loaded correctly.\n",trainedModelLabelsFullName.c_str());
    CD_SUCCESS("%d labels have been loaded.\n",labelsMap.size());;

    valid = true; // setTensorShape at firstArrived
}

/*****************************************************************/

void TensorFlowDetector::setTensorShape(tensorflow::int64 h, tensorflow::int64 w)
{
    CD_DEBUG("%d, %d\n",h,w);
    shape = tensorflow::TensorShape();
    shape.AddDim(1);
    shape.AddDim(h);
    shape.AddDim(w);
    shape.AddDim(3);
}

/*****************************************************************/

bool TensorFlowDetector::detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                                std::vector<DetectedObject*>& detectedObjects)
{
    if(!firstArrived)
    {
        setTensorShape(inYarpImg.width(),inYarpImg.height());
        firstArrived = true;
    }

    cv::Mat inCvMat = cv::cvarrToMat((IplImage*)inYarpImg.getIplImage());
    cv::cvtColor(inCvMat, inCvMat, cv::COLOR_BGR2RGB);

    // Mat -> Tensor
    tensorflow::Tensor tensor = tensorflow::Tensor(tensorflow::DT_FLOAT, shape);
    tensorflow::Status readTensorStatus = readTensorFromMat(inCvMat, tensor);
    if (!readTensorStatus.ok())
    {
        CD_ERROR("Mat OpenCV -> Tensor: FAIL\n");
        return false;
    }

    // Execute graph
    std::vector<tensorflow::Tensor> outputs; // outputs.clear();
    tensorflow::Status runStatus = session->Run({{inputLayer, tensor}}, outputLayer, {}, &outputs);
    if (!runStatus.ok())
    {
        CD_ERROR("Running model status: FAIL\n");
        return false;
    }

    // Extract results
    tensorflow::TTypes<float>::Flat scores = outputs[1].flat<float>();
    tensorflow::TTypes<float>::Flat classes = outputs[2].flat<float>();
    tensorflow::TTypes<float, 3>::Tensor boxes = outputs[0].flat_outer_dims<float,3>();

    //CD_INFO("scores.size(): %d\n",scores.size()); // 100
    std::vector<size_t> goodIdxs = filterBoxes(scores, boxes, DEFAULT_THRESHOLD_IOU, DEFAULT_THRESHOLD_SCORE);
    //CD_INFO("goodIdxs.size(): %d\n",goodIdxs.size());

    for (size_t i = 0; i < goodIdxs.size(); i++)
    {
        CD_SUCCESS("Detection: \"%s\" -> Score: %f\n",labelsMap[classes(goodIdxs[i])].c_str(),scores(goodIdxs[i]));

        DetectedObject* detectedObject = new DetectedObject;
        double yMin = boxes(0,goodIdxs[i],0);
        double xMin = boxes(0,goodIdxs[i],1);
        double yMax = boxes(0,goodIdxs[i],2);
        double xMax = boxes(0,goodIdxs[i],3);
        int tlx = xMin * inYarpImg.width();
        int tly = yMin * inYarpImg.height();
        int brx = xMax * inYarpImg.width();
        int bry = yMax * inYarpImg.height();
        detectedObject->setBoundingBox(tlx,
                                       tly,
                                       brx,
                                       bry);
        detectedObjects.push_back(detectedObject);
    }

    return true;
}

/*****************************************************************/

tensorflow::Status TensorFlowDetector::loadGraph(const tensorflow::string &graph_file_name, std::unique_ptr<tensorflow::Session> *session)
{
    tensorflow::GraphDef graph_def;
    tensorflow::Status load_graph_status = ReadBinaryProto(tensorflow::Env::Default(), graph_file_name, &graph_def);
    if (!load_graph_status.ok())
    {
        return tensorflow::errors::NotFound("Fail loading graph: '", graph_file_name, "'");
    }
    session->reset(tensorflow::NewSession(tensorflow::SessionOptions()));
    tensorflow::Status session_create_status = (*session)->Create(graph_def);
    if (!session_create_status.ok())
    {
        return session_create_status;
    }
    return tensorflow::Status::OK();
}

/************************************************************************/

tensorflow::Status TensorFlowDetector::readLabelsMapFile(const tensorflow::string &fileName, std::map<int, tensorflow::string> &labelsMap)
{
    std::ifstream t(fileName);
    if (t.bad())
        return tensorflow::errors::NotFound("Fail loading labels: '", fileName, "'");
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::string fileString = buffer.str();

    // Search entry patterns of type 'item { ... }' and parse each of them
    std::smatch matcherEntry;
    std::smatch matcherId;
    std::smatch matcherName;
    const std::regex reEntry("item \\{([\\S\\s]*?)\\}");
    const std::regex reName("\'.+\'");
    const std::regex reId("[0-9]+");
    std::string entry;

    auto stringBegin = std::sregex_iterator(fileString.begin(), fileString.end(), reEntry);
    auto stringEnd = std::sregex_iterator();

    int id;
    std::string name;
    for (std::sregex_iterator i = stringBegin; i != stringEnd; i++)
    {
        matcherEntry = *i;
        entry = matcherEntry.str();
        std::regex_search(entry, matcherId, reId);
        if (!matcherId.empty())
            id = stoi(matcherId[0].str());
        else
            continue;
        std::regex_search(entry, matcherName, reName);
        if (!matcherName.empty())
            name = matcherName[0].str().substr(1, matcherName[0].str().length() - 2);
        else
            continue;
        labelsMap.insert(std::pair<int, std::string>(id, name));
    }
    return tensorflow::Status::OK();
}

/************************************************************************/

tensorflow::Status TensorFlowDetector::readTensorFromMat(const cv::Mat &mat, tensorflow::Tensor &outTensor)
{
    cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
    auto root = tensorflow::Scope::NewRootScope();
    using namespace ::tensorflow::ops;

    // Trick from https://github.com/tensorflow/tensorflow/issues/8033
    float *p = outTensor.flat<float>().data();
    cv::Mat fakeMat(mat.rows, mat.cols, CV_32FC3, p);
    mat.convertTo(fakeMat, CV_32FC3);

    auto input_tensor = Placeholder(root.WithOpName("input"), tensorflow::DT_FLOAT);
    std::vector<std::pair<std::string, tensorflow::Tensor>> inputs = {{"input", outTensor}};
    auto uint8Caster = Cast(root.WithOpName("uint8_Cast"), outTensor, tensorflow::DT_UINT8);

    // Tensor output
    tensorflow::GraphDef graph;
    TF_RETURN_IF_ERROR(root.ToGraphDef(&graph));

    std::vector<tensorflow::Tensor> outTensors;
    std::unique_ptr<tensorflow::Session> session(tensorflow::NewSession(tensorflow::SessionOptions()));

    TF_RETURN_IF_ERROR(session->Create(graph));
    TF_RETURN_IF_ERROR(session->Run({inputs}, {"uint8_Cast"}, {}, &outTensors));

    outTensor = outTensors.at(0);
    cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
    return tensorflow::Status::OK();
}

/************************************************************************/

void TensorFlowDetector::drawBoundingBoxOnImage(cv::Mat &image, double yMin, double xMin, double yMax, double xMax,
                            double score, std::string label, bool scaled=true)
{
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    cv::Point tl, br;
    if (scaled)
    {
        tl = cv::Point((int) (xMin * image.cols), (int) (yMin * image.rows));
        br = cv::Point((int) (xMax * image.cols), (int) (yMax * image.rows));
    }
    else
    {
        tl = cv::Point((int) xMin, (int) yMin);
        br = cv::Point((int) xMax, (int) yMax);
    }
    cv::rectangle(image, tl, br, cv::Scalar(0, 255, 255), 1);

    float scoreRounded = floorf(score * 1000) / 1000;
    std::string scoreString = std::to_string(scoreRounded).substr(0, 5);
    std::string caption = label + " (" + scoreString + ")";

    int fontCoeff = 12;
    cv::Point brRect = cv::Point(tl.x + caption.length() * fontCoeff / 1.6, tl.y + fontCoeff);
    cv::rectangle(image, tl, brRect, cv::Scalar(0, 255, 255), -1);
    cv::Point textCorner = cv::Point(tl.x, tl.y + fontCoeff * 0.9);
    cv::putText(image, caption, textCorner, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0));
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
}

/************************************************************************/

void TensorFlowDetector::drawBoundingBoxesOnImage(cv::Mat &image,
                              tensorflow::TTypes<float>::Flat &scores,
                              tensorflow::TTypes<float>::Flat &classes,
                              tensorflow::TTypes<float,3>::Tensor &boxes,
                              std::map<int, tensorflow::string> &labelsMap,
                              std::vector<size_t> &idxs)
{
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    for (int j = 0; j < idxs.size(); j++)
        drawBoundingBoxOnImage(image,
                               boxes(0,idxs.at(j),0), boxes(0,idxs.at(j),1),
                               boxes(0,idxs.at(j),2), boxes(0,idxs.at(j),3),
                               scores(idxs.at(j)), labelsMap[classes(idxs.at(j))]);
                               cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
}

/************************************************************************/

double TensorFlowDetector::IOU(cv::Rect2f box1, cv::Rect2f box2)
{
    float xA = std::max(box1.tl().x, box2.tl().x);
    float yA = std::max(box1.tl().y, box2.tl().y);
    float xB = std::min(box1.br().x, box2.br().x);
    float yB = std::min(box1.br().y, box2.br().y);

    float intersectArea = abs((xB - xA) * (yB - yA));
    float unionArea = abs(box1.area()) + abs(box2.area()) - intersectArea;

    return 1. * intersectArea / unionArea;
}

/************************************************************************/

std::vector<size_t> TensorFlowDetector::filterBoxes(tensorflow::TTypes<float>::Flat &scores,
                                tensorflow::TTypes<float, 3>::Tensor &boxes,
                                double thresholdIOU, double thresholdScore)
{
    //CD_DEBUG("thresholdIOU: %f; thresholdScore: %f\n",thresholdIOU,thresholdScore);

    std::vector<size_t> sortIdxs(scores.size());
    iota(sortIdxs.begin(), sortIdxs.end(), 0);

    std::set<size_t> badIdxs = std::set<size_t>();
    size_t i = 0;
    while (i < sortIdxs.size())
    {
        if (scores(sortIdxs.at(i)) < thresholdScore)
            badIdxs.insert(sortIdxs[i]);

        if (badIdxs.find(sortIdxs.at(i)) != badIdxs.end())
        {
            i++;
            continue;
        }

        cv::Rect2f box1 = cv::Rect2f(cv::Point2f(boxes(0, sortIdxs.at(i), 1), boxes(0, sortIdxs.at(i), 0)),
                                     cv::Point2f(boxes(0, sortIdxs.at(i), 3), boxes(0, sortIdxs.at(i), 2)));

        for (size_t j = i + 1; j < sortIdxs.size(); j++)
        {
            if (scores(sortIdxs.at(j)) < thresholdScore)
            {
                badIdxs.insert(sortIdxs[j]);
                continue;
            }
            cv::Rect2f box2 = cv::Rect2f(cv::Point2f(boxes(0, sortIdxs.at(j), 1), boxes(0, sortIdxs.at(j), 0)),
                                         cv::Point2f(boxes(0, sortIdxs.at(j), 3), boxes(0, sortIdxs.at(j), 2)));
            if (IOU(box1, box2) > thresholdIOU)
                badIdxs.insert(sortIdxs[j]);
        }
        i++;
    }

    std::vector<size_t> goodIdxs = std::vector<size_t>();

    for (auto it = sortIdxs.begin(); it != sortIdxs.end(); it++)
        if (badIdxs.find(sortIdxs.at(*it)) == badIdxs.end())
            goodIdxs.push_back(*it);

    return goodIdxs;
}

/************************************************************************/

}// namespace roboticslab
