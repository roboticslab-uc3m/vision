/*
 * ************************************************************
 *      Program: Tensorflow Detection 2D Module
 *      Type: tensorflowdetection2d.cpp
 *      Author: David Velasco Garcia @davidvelascogarcia
 * ************************************************************
 */

// Libraries

#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Image.h>
#include "TensorflowDetection2D.hpp"
#include "TensorflowDetector.hpp"
#include "MainDetector.hpp"


tensorflowDetection2D::tensorflowDetection2D()
{
}

void tensorflowDetection2D::init(std::string labels, std::string graph)
{
  vgg16_graph=graph;
  vgg16_labels=labels;
}

int tensorflowDetection2D::detector(yarp::os::Port sender_port_post, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *inImg){

  inImg_i=inImg;
  maindetector detection_module;
  detection_module.detect(vgg16_labels, vgg16_graph, sender_port_post, inImg_i);
  return 0;
}
