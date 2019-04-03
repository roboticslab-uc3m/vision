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
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/sig/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.hpp>
#include <time.h>
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/util/command_line_flags.h"
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

cv::Mat tensorflowDetection2D::get_image()
{
    cv::VideoCapture webcam;
    cv::Mat picture;
    // Abrir cámara
    int cam_ok=0;
    while(cam_ok==0){
    if(!webcam.open(cam_path)){
      std::cout << "I can´t open source video, check connection..." << std::endl;
    }else{
        std::cout<<"Source video opended correctly"<<std::endl;
        cam_ok=1;
    }}

    webcam >> picture;

    return picture;

}
int tensorflowDetection2D::detector(yarp::os::Port sender_port_post, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *inImg){

  inImg_i=inImg;
  maindetector detection_module;
  detection_module.detect(vgg16_labels, vgg16_graph, sender_port_post, inImg_i);
  return 0;
}
