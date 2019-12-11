// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TENSOR_FLOW_DETECTOR_HELPER_HPP__
#define __TENSOR_FLOW_DETECTOR_HELPER_HPP__

#include <opencv2/core/mat.hpp>

#include <tensorflow/core/framework/tensor.h>
#include <tensorflow/core/public/session.h>

tensorflow::Status readLabelsMapFile(const tensorflow::string &fileName, std::map<int, tensorflow::string> &labelsMap);

tensorflow::Status loadGraph(const tensorflow::string &graph_file_name,
                             std::unique_ptr<tensorflow::Session> *session);

tensorflow::Status readTensorFromMat(const cv::Mat &mat, tensorflow::Tensor &outTensor);

void drawBoundingBoxOnImage(cv::Mat &image, double xMin, double yMin, double xMax, double yMax,
                            double score, std::string label, bool scaled);

void drawBoundingBoxesOnImage(cv::Mat &image,
                              tensorflow::TTypes<float>::Flat &scores,
                              tensorflow::TTypes<float>::Flat &classes,
                              tensorflow::TTypes<float,3>::Tensor &boxes,
                              std::map<int, tensorflow::string> &labelsMap,
                              std::vector<size_t> &idxs);

double IOU(cv::Rect box1, cv::Rect box2);

std::vector<size_t> filterBoxes(tensorflow::TTypes<float>::Flat &scores,
                                tensorflow::TTypes<float, 3>::Tensor &boxes,
                                double thresholdIOU, double thresholdScore);

#endif // __TENSORFLOW_DETECTOR_HELPER_HPP__
