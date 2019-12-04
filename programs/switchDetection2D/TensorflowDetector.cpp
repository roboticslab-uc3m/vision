
// Libraries

#include <fstream>
#include <iostream>
#include <math.h>
#include <regex>
#include <utility>
#include <vector>

#include <cv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"

#include "TensorflowDetector.hpp"

/************************************************************************/
// Create session and load graph
/************************************************************************/

tensorflow::Status loadGraph(const tensorflow::string &graph_file_name,
              std::unique_ptr<tensorflow::Session> *session) {
    tensorflow::GraphDef graph_def;
    tensorflow::Status load_graph_status =
            ReadBinaryProto(tensorflow::Env::Default(), graph_file_name, &graph_def);
    if (!load_graph_status.ok()) {
        return tensorflow::errors::NotFound("Fail loading graph: '",
                                            graph_file_name, "'");
    }
    session->reset(tensorflow::NewSession(tensorflow::SessionOptions()));
    tensorflow::Status session_create_status = (*session)->Create(graph_def);
    if (!session_create_status.ok()) {
        return session_create_status;
    }
    return tensorflow::Status::OK();
}

/************************************************************************/
// Read labels
/************************************************************************/

tensorflow::Status readLabelsMapFile(const tensorflow::string &fileName, std::map<int, tensorflow::string> &labelsMap) {


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
    for (std::sregex_iterator i = stringBegin; i != stringEnd; i++) {
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
//  Mat OpenCV -> TensorFlow
/************************************************************************/

tensorflow::Status readTensorFromMat(const cv::Mat &mat, tensorflow::Tensor &outTensor) {
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

void drawBoundingBoxOnImage(cv::Mat &image, double yMin, double xMin, double yMax, double xMax, double score, std::string label, bool scaled=true) {
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    cv::Point tl, br;
    if (scaled) {
        tl = cv::Point((int) (xMin * image.cols), (int) (yMin * image.rows));
        br = cv::Point((int) (xMax * image.cols), (int) (yMax * image.rows));
    } else {
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
// Draw on image
/************************************************************************/

void drawBoundingBoxesOnImage(cv::Mat &image,
                              tensorflow::TTypes<float>::Flat &scores,
                              tensorflow::TTypes<float>::Flat &classes,
                              tensorflow::TTypes<float,3>::Tensor &boxes,
                              std::map<int, tensorflow::string> &labelsMap,
                              std::vector<size_t> &idxs) {
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    for (int j = 0; j < idxs.size(); j++)
        drawBoundingBoxOnImage(image,
                               boxes(0,idxs.at(j),0), boxes(0,idxs.at(j),1),
                               boxes(0,idxs.at(j),2), boxes(0,idxs.at(j),3),
                               scores(idxs.at(j)), labelsMap[classes(idxs.at(j))]);
                               cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
}

/************************************************************************/
double IOU(cv::Rect2f box1, cv::Rect2f box2) {

    float xA = std::max(box1.tl().x, box2.tl().x);
    float yA = std::max(box1.tl().y, box2.tl().y);
    float xB = std::min(box1.br().x, box2.br().x);
    float yB = std::min(box1.br().y, box2.br().y);

    float intersectArea = abs((xB - xA) * (yB - yA));
    float unionArea = abs(box1.area()) + abs(box2.area()) - intersectArea;

    return 1. * intersectArea / unionArea;
}

/************************************************************************/
std::vector<size_t> filterBoxes(tensorflow::TTypes<float>::Flat &scores,
                           tensorflow::TTypes<float, 3>::Tensor &boxes,
                           double thresholdIOU, double thresholdScore) {

    std::vector<size_t> sortIdxs(scores.size());
    iota(sortIdxs.begin(), sortIdxs.end(), 0);

    std::set<size_t> badIdxs = std::set<size_t>();
    size_t i = 0;
    while (i < sortIdxs.size()) {
        if (scores(sortIdxs.at(i)) < thresholdScore)
            badIdxs.insert(sortIdxs[i]);
        if (badIdxs.find(sortIdxs.at(i)) != badIdxs.end()) {
            i++;
            continue;
        }

        cv::Rect2f box1 = cv::Rect2f(cv::Point2f(boxes(0, sortIdxs.at(i), 1), boxes(0, sortIdxs.at(i), 0)),
                             cv::Point2f(boxes(0, sortIdxs.at(i), 3), boxes(0, sortIdxs.at(i), 2)));
        for (size_t j = i + 1; j < sortIdxs.size(); j++) {
            if (scores(sortIdxs.at(j)) < thresholdScore) {
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
